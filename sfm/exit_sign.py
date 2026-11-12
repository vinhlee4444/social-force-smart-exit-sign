"""
ExitSign: one smart exit sign = camera + crowd counter + gate controller.

Each sign works completely on its own, like the real device:
  1. `count(pos)`  : how many people the camera can see. The visible area is a CONE
                     (SensorConfig.fov_deg) facing the corridor, up to `radius` metres,
                     minus the blind zone below the camera (mount height + tilt), with a
                     clear line of sight (no seeing through walls).
  2. `update(t, count)` : turns the count into a metric (density or count), smooths it with a
                     moving average, and feeds the gate state machine

        OPEN   --(value > close_above)------------------------------> CLOSED
        CLOSED --(closed >= min_closed_time AND value <= reopen_at_most)--> OPEN

The crowd-counting model is assumed to be perfect, so `count` is the exact number of
agents whose centre lies inside the camera's visible area.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

from .config import ExitConfig, GateConfig, SensorConfig
from .floorplan import FloorPlan


@dataclass
class GateEvent:
    t: float
    closed: bool
    count: float                 # the (smoothed) value the controller read
    reason: str = ""


class GateController:
    """Pure logic, no geometry: easy to unit-test.

        OPEN   --(value > close_above, open >= min_open_time)-----------> CLOSED
        CLOSED --(closed >= min_closed_time AND value <= reopen_at_most)-> OPEN
        CLOSED --(closed >= max_closed_time)-----------------------------> OPEN   (fail-safe)

    `min_open_time = 0` and `max_closed_time = inf` switch the two extra safeguards off.
    """

    def __init__(self, cfg: GateConfig):
        self.cfg = cfg
        self.closed = False
        self.t_changed = -np.inf          # last time the gate closed / reopened
        self.t_opened = -np.inf           # last time it (re)opened
        self.events: List[GateEvent] = []

    @property
    def state(self) -> str:
        return "CLOSED" if self.closed else "OPEN"

    def update(self, t: float, value: float) -> bool:
        """Feed one sensor reading. Returns True if the gate changed state (closed or reopened)."""
        c, eps = self.cfg, 1e-9
        if not c.enabled:
            return False
        if not self.closed:
            if value > c.close_above and t - self.t_opened >= c.min_open_time - eps:
                return self._set(True, t, value, "threshold")
        else:
            waited = t - self.t_changed
            if waited >= c.max_closed_time - eps:
                return self._set(False, t, value, "max_closed_time")
            if waited >= c.min_closed_time - eps and value <= c.reopen_at_most:
                return self._set(False, t, value, "crowd_cleared")
        return False

    def _set(self, closed: bool, t: float, value: float, reason: str) -> bool:
        self.closed = closed
        self.t_changed = t
        if not closed:
            self.t_opened = t
        self.events.append(GateEvent(t, closed, value, reason))
        return True


class ExitSign:
    def __init__(self, idx: int, ex: ExitConfig, floor: FloorPlan,
                 sensor: SensorConfig, gate: GateConfig, evac_depth: float,
                 rng: Optional[np.random.Generator] = None):
        self.rng = rng if rng is not None else np.random.default_rng(idx)
        self.idx = idx
        self.name = ex.name
        self.cfg = ex
        self.sensor = sensor
        self.floor = floor
        self.gate = GateController(gate)
        self.evac_depth = evac_depth

        self.door_m = floor.rect_to_m(ex.door)
        self.room_m = floor.rect_to_m(ex.room)
        dx0, dy0, dx1, dy1 = self.door_m
        self.inward = np.array(ex.inward, float)
        self.door_center = np.array([(dx0 + dx1) / 2, (dy0 + dy1) / 2])
        # Camera sits on the exit sign above the door, on the corridor side of the wall.
        half_depth = abs(np.dot([dx1 - dx0, dy1 - dy0], self.inward)) / 2
        self.camera = self.door_center - self.inward * (half_depth + 0.2)
        self.facing = -self.inward                         # looks from the door into the corridor
        self.min_range, self.max_range = self._ground_range()
        self.mask = self._visible_mask()
        self.visible_area_m2 = float(self.mask.sum()) * floor.s ** 2
        self.last_count = 0                                # raw people count in the cone
        self.last_value = 0.0                              # smoothed metric the gate reads
        n_win = max(1, int(round(sensor.smooth_window / sensor.period))) if sensor.smooth_window > 0 else 1
        self._window: Deque[float] = deque(maxlen=n_win)

    # ------------------------------------------------------------------ mounting geometry
    def _ground_range(self) -> Tuple[float, float]:
        """(near, far) horizontal distance from the camera that its lens covers on the floor.

        Camera at height h, pitched `tilt` below horizontal, vertical FOV v:
          lowest ray  = tilt + v/2 -> hits the floor at h / tan(tilt + v/2)   (start of coverage)
          highest ray = tilt - v/2 -> hits the floor at h / tan(tilt - v/2)   (end; infinite if <= 0)
        Floor level is used (conservative): heads are visible a bit closer than feet.
        """
        sc = self.sensor
        if not sc.use_min_range or sc.mount_height <= 0 or sc.vfov_deg <= 0:
            return 0.0, float("inf")
        a_near = np.radians(sc.tilt_deg + sc.vfov_deg / 2)
        a_far = np.radians(sc.tilt_deg - sc.vfov_deg / 2)
        near = sc.mount_height / np.tan(a_near) if a_near < np.pi / 2 else 0.0
        far = sc.mount_height / np.tan(a_far) if a_far > 1e-6 else float("inf")
        return float(near), float(far)

    # ------------------------------------------------------------------ sensing
    def _visible_mask(self) -> np.ndarray:
        """Floor pixels the camera can actually see: inside the cone, between the blind
        zone and the maximum range, with a clear line of sight."""
        fl, s = self.floor, self.floor.s
        R_px = min(self.sensor.radius, self.max_range) / s
        near_px = self.min_range / s
        cx, cy = self.camera / s                                  # camera, pixel units
        x0, x1 = int(max(0, cx - R_px)), int(min(fl.W - 1, cx + R_px))
        y0, y1 = int(max(0, cy - R_px)), int(min(fl.H - 1, cy + R_px))
        ys, xs = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        px, py = xs + 0.5, ys + 0.5                               # pixel centres
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        inside = (d2 <= R_px ** 2) & (d2 >= near_px ** 2) & fl.base_free[ys, xs]
        # The sign faces the hallway: exclude the exit room itself.
        rx0, ry0, rx1, ry1 = self.cfg.room
        in_exit_room = (xs >= rx0) & (xs <= rx1) & (ys >= ry0) & (ys <= ry1)
        inside &= ~in_exit_room
        if self.sensor.fov_deg < 360.0:                           # keep only the cone
            ex, ey = px - cx, py - cy
            cosang = (ex * self.facing[0] + ey * self.facing[1]) / (np.sqrt(d2) + 1e-9)
            inside &= cosang >= np.cos(np.radians(self.sensor.fov_deg / 2))
        mask = np.zeros((fl.H, fl.W), bool)
        tx, ty = px[inside], py[inside]
        if not self.sensor.line_of_sight:
            mask[ys[inside], xs[inside]] = True
            return mask
        # Ray-march from the camera to every candidate pixel (2 samples per pixel length).
        n_samp = int(np.ceil(2 * R_px)) + 1
        visible = np.ones(len(tx), bool)
        free = fl.base_free
        for k in range(1, n_samp):
            a = k / n_samp
            sx = (cx + a * (tx - cx)).astype(np.int64)
            sy = (cy + a * (ty - cy)).astype(np.int64)
            visible &= free[sy, sx]
        mask[ys[inside][visible], xs[inside][visible]] = True
        return mask

    def count(self, pos: np.ndarray) -> int:
        if len(pos) == 0:
            return 0
        px, py = self.floor.to_px(pos)
        return int(self.mask[py, px].sum())

    def update(self, t: float, count: int) -> bool:
        """One camera reading -> (AI counting error) -> metric -> moving average -> gate."""
        if self.sensor.count_error > 0 and count > 0:
            count = max(0, int(round(count * (1.0 + self.rng.normal(0.0, self.sensor.count_error)))))
        self.last_count = count
        if self.sensor.metric == "density":
            raw = count / self.visible_area_m2 if self.visible_area_m2 > 0 else 0.0
        else:
            raw = float(count)
        self._window.append(raw)
        self.last_value = float(np.mean(self._window))
        return self.gate.update(t, self.last_value)

    @property
    def closed(self) -> bool:
        return self.gate.closed

    @property
    def state(self) -> str:
        return self.gate.state

    # ------------------------------------------------------------------ geometry
    def _depth_past_door(self, pos: np.ndarray) -> np.ndarray:
        """Signed distance past the INNER edge of the door, into the exit room."""
        dx0, dy0, dx1, dy1 = self.door_m
        ix, iy = self.inward
        if iy < 0:
            return dy0 - pos[:, 1]
        if iy > 0:
            return pos[:, 1] - dy1
        if ix < 0:
            return dx0 - pos[:, 0]
        return pos[:, 0] - dx1

    def _in_room_footprint(self, pos: np.ndarray) -> np.ndarray:
        rx0, ry0, rx1, ry1 = self.room_m
        return (pos[:, 0] >= rx0) & (pos[:, 0] <= rx1) & (pos[:, 1] >= ry0) & (pos[:, 1] <= ry1)

    def evacuated(self, pos: np.ndarray) -> np.ndarray:
        """Agents that are deep enough inside this exit room count as safe."""
        return self._in_room_footprint(pos) & (self._depth_past_door(pos) >= self.evac_depth)

    def gate_segment(self) -> Tuple[float, float, float, float]:
        return self.door_m

