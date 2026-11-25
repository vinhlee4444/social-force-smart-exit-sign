"""Conservative advice using mapped routes, camera coverage and prior broadcasts."""
from __future__ import annotations

from itertools import product

import numpy as np


def branch_split(old, new, heading, min_shared, lookahead):
    """Find a nearby split that leaves both routes going forward."""
    size = min(len(old), len(new))
    differences = np.flatnonzero((old[:size] != new[:size]).any(axis=1))
    if not len(differences):
        return None
    split = differences[0] - 1
    if split < 1:
        return None
    shared = np.linalg.norm(np.diff(old[:split + 1], axis=0), axis=1).sum()
    if not min_shared <= shared <= lookahead:
        return None
    incoming = old[split] - old[0]
    if incoming @ heading <= 0:
        return None
    # Reject a branch pointing back along its incoming leg. Check the next
    # metre of both paths so a single raster-grid turn does not decide this.
    for path in (old, new):
        lengths = np.cumsum(np.linalg.norm(np.diff(path[split:], axis=0), axis=1))
        stop = min(np.searchsorted(lengths, min_shared), len(lengths) - 1) + split + 1
        outgoing = path[stop] - path[split]
        if outgoing @ incoming < -0.2 * np.linalg.norm(outgoing) * np.linalg.norm(incoming):
            return None
    return split


def branch_window(old, new, heading, min_shared, lookahead):
    return branch_split(old, new, heading, min_shared, lookahead) is not None


def shared_start_cell(old, new, point):
    cell = old.cell
    fx, fy = point / cell - 0.5
    x, y = int(np.floor(fx)), int(np.floor(fy))
    h, w = old.dist.shape
    candidates = [(iy, ix) for iy in (y, y + 1) for ix in (x, x + 1)
                  if 0 <= iy < h and 0 <= ix < w
                  and np.isfinite(old.dist[iy, ix]) and np.isfinite(new.dist[iy, ix])]
    return min(candidates, key=lambda ij: np.hypot(ij[0] - fy, ij[1] - fx), default=None)


def before_forward_branch(old, new, point, heading, cfg):
    start = shared_start_cell(old, new, point)
    if start is None:
        return False
    length = cfg.branch_lookahead_m + cfg.branch_min_shared_m
    return branch_window(old.path_prefix(start, length), new.path_prefix(start, length),
                         heading, cfg.branch_min_shared_m, cfg.branch_lookahead_m)


def corridor_segments(path):
    """Sample a mapped polyline in segments of at most half a metre."""
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    along = np.r_[np.arange(0, lengths[-1], 0.5), lengths[-1]]
    pts = np.column_stack([np.interp(along, lengths, path[:, axis]) for axis in range(2)])
    vectors = np.diff(pts, axis=0)
    norms = np.linalg.norm(vectors, axis=1)
    keep = norms > 1e-6
    return (0.5 * (pts[:-1] + pts[1:]))[keep], vectors[keep] / norms[keep, None]


def opposing_corridor(first, second, floor, width):
    """Detect opposite near-parallel legs with overlapping span and clear floor between them."""
    points, directions = first
    other, other_dirs = second
    offsets = other[None] - points[:, None]
    close = np.einsum('ijk,ijk->ij', offsets, offsets) <= width ** 2
    facing = directions @ other_dirs.T <= -0.7
    overlap = np.abs(np.einsum('ijk,ik->ij', offsets, directions)) <= 0.5
    rows, cols = np.nonzero(close & facing & overlap)
    return any(floor.visible_from(points[i], other[j:j+1])[0] for i, j in zip(rows, cols))


class RouteCompatible:
    """Track mapped directions that the hub has advised, with no live corridor counts."""

    def __init__(self, floor, cfg):
        self.floor = floor
        self.cfg = cfg
        self.issued_paths = {}
        self.last_issued = {}

    def check(self, old, new, point, heading):
        start = shared_start_cell(old, new, point)
        if start is None:
            return None, 'unmapped_route'
        old_path = old.path_prefix(start, np.inf)
        new_path = new.path_prefix(start, np.inf)
        for nav, path in ((old, old_path), (new, new_path)):
            x, y = np.rint(path[-1] / nav.cell).astype(int)
            if nav.dist[y, x] != 0:
                return None, 'unmapped_route'
        cfg = self.cfg
        split = branch_split(old_path, new_path, heading, cfg.branch_min_shared_m, cfg.branch_lookahead_m)
        if split is None:
            return None, 'no_forward_branch'
        # The graph stores raster indices. Move to the cell centres for line-of-sight checks.
        alternate = corridor_segments(new_path[split:] + 0.5 * new.cell)
        original = corridor_segments(old_path + 0.5 * old.cell)
        width = cfg.opposing_corridor_width_m
        if (opposing_corridor(alternate, original, self.floor, width)
                or opposing_corridor(alternate, alternate, self.floor, width)):
            return None, 'opposes_mapped_route'
        if any(opposing_corridor(alternate, path, self.floor, width) for path in self.issued_paths.values()):
            return None, 'opposes_issued_route'
        return (new_path[split:].tobytes(), alternate), None

    def issue(self, source, destination, route, t):
        key, segments = route
        # No camera currently confirms that the whole corridor has cleared.
        self.issued_paths.setdefault(key, segments)
        self.last_issued[source, destination] = t

    def displayed(self, source, options, t):
        return tuple(dest for dest in options if
                     t - self.last_issued.get((source, dest), -np.inf) <= self.cfg.decision_recheck_period)


class CoveredRoutes:
    """Abstain on unobserved approaches and avoid reversing an advertised flow."""

    def __init__(self, sim):
        self.sim = sim
        self.coverage = [self._coverage(sign, nav) for sign, nav in zip(sim.signs, sim.nav)]
        self.advertised = set()
        self.routes = {}
        self.broadcasts = {}

    def publish(self, network, t):
        """Reserve mapped directions when the hub issues recommendations."""
        self.broadcasts.clear()
        for source in range(len(network.states)):
            candidates = network.recommendations(source, t)
            if candidates:
                options, reason = self.filter(source, candidates)
                self.broadcasts[source] = options, reason
                for destination in options:
                    self.reserve(source, destination)

    def _coverage(self, sign, nav):
        y, x = np.indices(nav.dist.shape)
        pos = (np.stack((x, y), axis=-1) + 0.5) * nav.cell
        door_depth = nav.distance(sign.camera[None])[0]
        xmin, ymin, xmax, ymax = sign.room_m
        in_room = ((pos[..., 0] >= xmin) & (pos[..., 0] <= xmax)
                   & (pos[..., 1] >= ymin) & (pos[..., 1] <= ymax))
        required = np.isfinite(nav.dist) & ~in_room
        required &= nav.dist <= door_depth + self.sim.cfg.guidance.approach_length
        px, py = self.sim.floor.to_px(pos[required])
        return float(sign.mask[py, px].mean()) if len(px) else 0.0

    def _route(self, source, destination):
        """Directed 1 m blocks on a mapped shortest path between door approaches."""
        key = source, destination
        if key in self.routes:
            return self.routes[key]
        nav = self.sim.nav[destination]
        point = self.sim.signs[source].camera
        x, y = (point / nav.cell).astype(int)
        dist = nav.dist
        cells = set()
        while np.isfinite(dist[y, x]) and dist[y, x] > 0:
            current = dist[y, x]
            choices = []
            for dy, dx in product((-1, 0, 1), repeat=2):
                ny, nx = y + dy, x + dx
                if not (0 <= ny < dist.shape[0] and 0 <= nx < dist.shape[1]):
                    continue
                if dist[ny, nx] >= current or not np.isfinite(dist[y, nx] + dist[ny, x]):
                    continue
                choices.append((dist[ny, nx] + np.hypot(dx, dy) * nav.cell, ny, nx))
            if not choices:
                break
            _, ny, nx = min(choices)
            a = tuple((np.array([x, y]) * nav.cell).astype(int))
            b = tuple((np.array([nx, ny]) * nav.cell).astype(int))
            if a != b:
                cells.add((a, b))
            y, x = ny, nx
        self.routes[key] = frozenset(cells) if dist[y, x] == 0 else frozenset()
        return self.routes[key]

    def filter(self, source, candidates):
        covered = [k for k in candidates if self.coverage[k] == 1.0]
        if not covered:
            return (), "approach_unobserved"
        allowed = tuple(k for k in covered if self._route(source, k)
                        and not any((b, a) in self.advertised for a, b in self._route(source, k)))
        return allowed, "unmapped_or_opposing_advertisement"

    def reserve(self, source, destination):
        # Camera data cannot tell the hub when the entire route has cleared.
        # Keep the direction of an issued recommendation for this run.
        self.advertised.update(self._route(source, destination))
