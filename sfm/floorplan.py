"""
FloorPlan: turns the floor-plan image into everything the physics needs.

* `free`      : bool image, True = walkable floor (current gate states applied)
* `sdf`       : signed distance to the nearest wall SURFACE in metres
                (> 0 on the floor, < 0 inside a wall). Continuous across the wall surface,
                so the wall normal is well defined even if someone gets pushed into a wall.
* `wall_normal(pos)` / `wall_distance(pos)` : bilinear look-ups used by the wall force.
* `rooms`     : label image of the rooms (used to spawn agents), detected automatically.
"""
from __future__ import annotations

import os
from typing import List, Tuple

import cv2
import numpy as np

from .config import ExitConfig, MapConfig, Rect


def bilinear(grid: np.ndarray, fx: np.ndarray, fy: np.ndarray) -> np.ndarray:
    """Sample `grid` at fractional (column, row) indices with bilinear interpolation.

    Index (i, j) refers to the centre of cell grid[j, i]. Indices are clamped to the grid.
    """
    h, w = grid.shape
    fx = np.clip(fx, 0.0, w - 1.000001)
    fy = np.clip(fy, 0.0, h - 1.000001)
    x0 = fx.astype(np.int64)
    y0 = fy.astype(np.int64)
    tx = fx - x0
    ty = fy - y0
    g00 = grid[y0, x0]
    g10 = grid[y0, x0 + 1]
    g01 = grid[y0 + 1, x0]
    g11 = grid[y0 + 1, x0 + 1]
    return (g00 * (1 - tx) * (1 - ty) + g10 * tx * (1 - ty)
            + g01 * (1 - tx) * ty + g11 * tx * ty)


class FloorPlan:
    def __init__(self, map_cfg: MapConfig, exits: List[ExitConfig], base_dir: str = "."):
        self.cfg = map_cfg
        self.exits = exits
        self.s = map_cfg.meters_per_pixel

        path = map_cfg.image_path
        if not os.path.isabs(path):
            path = os.path.join(base_dir, path)
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Could not read floor plan '{path}'")

        free = img > map_cfg.wall_threshold
        for x0, y0, x1, y1 in map_cfg.seal_rects:
            free[y0:y1 + 1, x0:x1 + 1] = False
        if map_cfg.exit_door_width_m is not None:
            w_px = max(1, int(round(map_cfg.exit_door_width_m / map_cfg.meters_per_pixel)))
            for ex in exits:                         # wall off both sides of the gap, keep the centre
                x0, y0, x1, y1 = ex.door
                if ex.inward[1] != 0:                # horizontal door gap (wall runs along x)
                    c = (x0 + x1) // 2
                    lo, hi = c - w_px // 2, c - w_px // 2 + w_px - 1
                    free[y0:y1 + 1, x0:lo] = False
                    free[y0:y1 + 1, hi + 1:x1 + 1] = False
                else:                                # vertical door gap (wall runs along y)
                    c = (y0 + y1) // 2
                    lo, hi = c - w_px // 2, c - w_px // 2 + w_px - 1
                    free[y0:lo, x0:x1 + 1] = False
                    free[hi + 1:y1 + 1, x0:x1 + 1] = False
        self.base_free = free                       # all gates open
        self.H, self.W = free.shape
        self.width_m = self.W * self.s
        self.height_m = self.H * self.s

        self.gate_closed = [False] * len(exits)
        self._rebuild()
        self.sdf_open = self.sdf.copy()             # gates-open SDF, used by static planners

    # ------------------------------------------------------------------ geometry
    def rect_to_m(self, r: Rect) -> Tuple[float, float, float, float]:
        """Pixel rect (inclusive) -> metric box (x0, y0, x1, y1) covering those pixels."""
        x0, y0, x1, y1 = r
        return x0 * self.s, y0 * self.s, (x1 + 1) * self.s, (y1 + 1) * self.s

    def _rebuild(self) -> None:
        free = self.base_free.copy()
        for closed, ex in zip(self.gate_closed, self.exits):
            if closed:
                x0, y0, x1, y1 = ex.door
                free[y0:y1 + 1, x0 - 1:x1 + 2] = False   # 1-px overlap with the door jambs
        self.free = free

        f8 = free.astype(np.uint8)
        d_free = cv2.distanceTransform(f8, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)       # floor px -> wall px
        d_wall = cv2.distanceTransform(1 - f8, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)   # wall px -> floor px
        # Pixel-centre distances -> distance to the wall surface (half a pixel closer).
        sdf_px = np.where(free, d_free - 0.5, -(d_wall - 0.5))
        self.sdf = (sdf_px * self.s).astype(np.float64)
        gy, gx = np.gradient(self.sdf)
        norm = np.hypot(gx, gy) + 1e-12
        self.nx = gx / norm
        self.ny = gy / norm

    def set_gate(self, idx: int, closed: bool) -> bool:
        """Open/close a gate. Returns True if the state actually changed."""
        if self.gate_closed[idx] == closed:
            return False
        self.gate_closed[idx] = closed
        self._rebuild()
        return True

    # ------------------------------------------------------------------ look-ups
    def _frac_idx(self, pos: np.ndarray):
        return pos[:, 0] / self.s - 0.5, pos[:, 1] / self.s - 0.5

    def wall_distance(self, pos: np.ndarray) -> np.ndarray:
        fx, fy = self._frac_idx(pos)
        return bilinear(self.sdf, fx, fy)

    def wall_normal(self, pos: np.ndarray) -> np.ndarray:
        """Unit vector pointing AWAY from the nearest wall."""
        fx, fy = self._frac_idx(pos)
        n = np.stack([bilinear(self.nx, fx, fy), bilinear(self.ny, fx, fy)], axis=1)
        return n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)

    def to_px(self, pos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        px = np.clip((pos[:, 0] / self.s).astype(np.int64), 0, self.W - 1)
        py = np.clip((pos[:, 1] / self.s).astype(np.int64), 0, self.H - 1)
        return px, py

    # ------------------------------------------------------------------ rooms
    def detect_rooms(self, door_close_radius_px: int) -> np.ndarray:
        """Label image of the spawnable rooms (0 = not a room).

        Doors are 'closed' by eroding the floor with a disc slightly larger than half a
        door width. The floor then falls apart into one big corridor network + one blob
        per room. Each blob is grown back to the full room, and the corridor network and
        the exit rooms are discarded.
        """
        k = door_close_radius_px
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
        core = cv2.erode(self.base_free.astype(np.uint8), ker)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(core, connectivity=4)
        if n <= 1:
            return np.zeros_like(lab)

        # Grow every core blob back to the walls: each floor pixel gets the label of the
        # nearest core pixel (Voronoi assignment via distanceTransformWithLabels).
        _, nearest = cv2.distanceTransformWithLabels(
            (core == 0).astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        zero_ys, zero_xs = np.nonzero(core)
        # `nearest` indexes zero-pixels of the input (= core pixels) in raster order (1-based).
        core_labels = lab[zero_ys, zero_xs]
        full = core_labels[nearest - 1]
        full[~self.base_free] = 0

        corridor = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        full[full == corridor] = 0
        for ex in self.exits:
            x0, y0, x1, y1 = ex.room
            for lbl in np.unique(full[y0:y1 + 1, x0:x1 + 1]):
                full[full == lbl] = 0
        # Re-number 1..n_rooms
        uniq = np.unique(full)
        uniq = uniq[uniq != 0]
        remap = np.zeros(full.max() + 1, dtype=np.int32)
        remap[uniq] = np.arange(1, len(uniq) + 1)
        return remap[full]

