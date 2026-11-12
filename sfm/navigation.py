"""
Navigation fields: for every exit, the walking distance from every floor cell to that
exit, going AROUND walls (geodesic distance). The desired walking direction of an agent
is the direction in which this distance decreases fastest.

Straight-line steering ("walk towards the exit") gets agents stuck behind walls in a
building with rooms; a distance field never does.

Fields are computed once with Dijkstra on a coarse grid (NavConfig.cell_size) and cached
to `cache/` because pure-Python Dijkstra takes a few seconds per exit.
"""
from __future__ import annotations

import hashlib
import heapq
import math
import os
from typing import List

import cv2
import numpy as np

from .config import ExitConfig, NavConfig
from .floorplan import FloorPlan, bilinear

_SQ2 = math.sqrt(2.0)
_OFFSETS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, _SQ2), (-1, 1, _SQ2), (1, -1, _SQ2), (1, 1, _SQ2)]   # (dy, dx, length)


def _dijkstra(passable: np.ndarray, cost: np.ndarray, sources: np.ndarray) -> np.ndarray:
    """Multi-source Dijkstra on an 8-connected grid. Returns distance in cell units."""
    H, W = passable.shape
    # Pad with a ring of blocked cells -> no bounds checks in the inner loop.
    Hp, Wp = H + 2, W + 2
    pas = np.zeros((Hp, Wp), bool)
    pas[1:-1, 1:-1] = passable
    cst = np.ones((Hp, Wp))
    cst[1:-1, 1:-1] = cost
    src = np.zeros((Hp, Wp), bool)
    src[1:-1, 1:-1] = sources & passable

    pas_l = pas.ravel().tolist()
    cst_l = cst.ravel().tolist()
    dist = [math.inf] * (Hp * Wp)
    heap = []
    for u in np.flatnonzero(src).tolist():
        dist[u] = 0.0
        heap.append((0.0, u))
    heapq.heapify(heap)
    nbrs = [(dy * Wp + dx, dy * Wp, dx, L) for dy, dx, L in _OFFSETS]
    push, pop = heapq.heappush, heapq.heappop

    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue
        cu = cst_l[u]
        for off, offy, offx, L in nbrs:
            v = u + off
            if not pas_l[v]:
                continue
            if offy and offx and not (pas_l[u + offy] and pas_l[u + offx]):
                continue                                    # no cutting wall corners
            nd = d + L * 0.5 * (cu + cst_l[v])
            if nd < dist[v]:
                dist[v] = nd
                push(heap, (nd, v))

    out = np.array(dist).reshape(Hp, Wp)[1:-1, 1:-1]
    return out


def _descent_direction(D: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    """Unit vector field pointing down the distance field D (inf = blocked)."""
    H, W = D.shape
    Dp = np.full((H + 2, W + 2), np.inf)
    Dp[1:-1, 1:-1] = D
    ax = np.zeros_like(D)
    ay = np.zeros_like(D)
    finite = np.isfinite(D)
    for dy, dx, L in _OFFSETS:
        Dn = Dp[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
        ok = finite & np.isfinite(Dn)
        rate = np.where(ok, (D - np.where(ok, Dn, 0.0)) / L, 0.0)
        rate = np.maximum(rate, 0.0)
        ax += rate * dx / L
        ay += rate * dy / L
    n = np.hypot(ax, ay)
    good = n > 1e-9
    dx_field = np.where(good, ax / np.where(good, n, 1), 0.0)
    dy_field = np.where(good, ay / np.where(good, n, 1), 0.0)
    # Target cells (D == 0) have no descent: keep walking deeper into the exit room.
    tgt = finite & (D == 0)
    dx_field[tgt] = fallback[0]
    dy_field[tgt] = fallback[1]
    return np.stack([dx_field, dy_field])


class NavigationField:
    """Distance-to-exit + walking direction for ONE exit."""

    def __init__(self, dist_m: np.ndarray, direction: np.ndarray, cell: float):
        self.dist = dist_m              # (Hc, Wc) metres, inf = unreachable
        self.dir = direction            # (2, Hc, Wc) unit vectors
        self.cell = cell

    def _frac(self, pos):
        return pos[:, 0] / self.cell - 0.5, pos[:, 1] / self.cell - 0.5

    def direction(self, pos: np.ndarray) -> np.ndarray:
        fx, fy = self._frac(pos)
        d = np.stack([bilinear(self.dir[0], fx, fy), bilinear(self.dir[1], fx, fy)], axis=1)
        return d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-12)

    def distance(self, pos: np.ndarray) -> np.ndarray:
        """Walking distance to the exit (nearest of the 4 surrounding cells)."""
        fx, fy = self._frac(pos)
        Hc, Wc = self.dist.shape
        x0 = np.clip(np.floor(fx).astype(int), 0, Wc - 2)
        y0 = np.clip(np.floor(fy).astype(int), 0, Hc - 2)
        return np.minimum.reduce([self.dist[y0, x0], self.dist[y0, x0 + 1],
                                  self.dist[y0 + 1, x0], self.dist[y0 + 1, x0 + 1]])


def build_navigation(floor: FloorPlan, exits: List[ExitConfig], cfg: NavConfig,
                     base_dir: str = ".", verbose: bool = True) -> List[NavigationField]:
    f = max(1, int(round(cfg.cell_size / floor.s)))     # pixels per nav cell
    cell = f * floor.s
    Hc, Wc = math.ceil(floor.H / f), math.ceil(floor.W / f)

    # Cache key: everything that influences the result.
    h = hashlib.sha1()
    h.update(np.packbits(floor.base_free).tobytes())
    h.update(repr((cfg.cell_size, cfg.wall_clearance, cfg.wall_penalty, cfg.target_depth,
                   floor.s, [(e.room, e.door, e.inward) for e in exits], "v2")).encode())
    cache_dir = cfg.cache_dir if os.path.isabs(cfg.cache_dir) else os.path.join(base_dir, cfg.cache_dir)
    cache_file = os.path.join(cache_dir, f"nav_{h.hexdigest()[:16]}.npz")
    if os.path.exists(cache_file):
        data = np.load(cache_file)
        return [NavigationField(data[f"dist{i}"], data[f"dir{i}"], cell) for i in range(len(exits))]

    # Coarse grid: a cell is passable only if all its pixels are floor (preserves thin walls).
    pad_h, pad_w = Hc * f - floor.H, Wc * f - floor.W
    free_p = np.pad(floor.base_free, ((0, pad_h), (0, pad_w)))
    sdf_p = np.pad(floor.sdf_open, ((0, pad_h), (0, pad_w)), constant_values=-1.0)
    blocks = free_p.reshape(Hc, f, Wc, f)
    passable = blocks.all(axis=(1, 3))
    clearance = sdf_p.reshape(Hc, f, Wc, f).max(axis=(1, 3))
    # Cost grows near walls so routes keep some distance from walls and corners.
    closeness = np.clip(1.0 - clearance / cfg.wall_clearance, 0.0, 1.0)
    cost = 1.0 + cfg.wall_penalty * closeness ** 2

    cy, cx = np.mgrid[0:Hc, 0:Wc]
    cxm, cym = (cx + 0.5) * cell, (cy + 0.5) * cell        # cell centres, metres

    fields, save = [], {}
    for i, ex in enumerate(exits):
        if verbose:
            print(f"  computing route field for {ex.name} ...", flush=True)
        rx0, ry0, rx1, ry1 = floor.rect_to_m(ex.room)
        dx0, dy0, dx1, dy1 = floor.rect_to_m(ex.door)
        ix, iy = ex.inward
        # Depth of a point past the INNER edge of the door, measured into the room.
        if iy < 0:
            depth = dy0 - cym
        elif iy > 0:
            depth = cym - dy1
        elif ix < 0:
            depth = dx0 - cxm
        else:
            depth = cxm - dx1
        in_room = (cxm >= rx0) & (cxm <= rx1) & (cym >= ry0) & (cym <= ry1)
        sources = in_room & (depth >= cfg.target_depth) & passable
        if not sources.any():
            raise ValueError(f"Exit '{ex.name}': no target cells found inside its room rectangle")
        D = _dijkstra(passable, cost, sources) * cell
        direction = _descent_direction(D, fallback=np.array([ix, iy], float))
        fields.append(NavigationField(D, direction, cell))
        save[f"dist{i}"] = D
        save[f"dir{i}"] = direction

    os.makedirs(cache_dir, exist_ok=True)
    np.savez_compressed(cache_file, **save)
    return fields

