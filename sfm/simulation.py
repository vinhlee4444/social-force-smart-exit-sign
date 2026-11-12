"""
Simulation: owns the clock and calls every component in the right order.

One step (dt):
  1. every sensor period: each ExitSign counts the people it sees and updates its state
  2. Social Force Model forces + integration (Crowd.step; agents walk towards their nearest exit)
  3. remove agents that reached an exit room
  4. every frame period: record metrics
"""
from __future__ import annotations

import os
from typing import List, Optional

import numpy as np

from .config import Config
from .crowd import Crowd
from .exit_sign import ExitSign
from .floorplan import FloorPlan
from .metrics import Recorder
from .navigation import NavigationField, build_navigation


class Simulation:
    def __init__(self, cfg: Config, base_dir: str = ".", floor: Optional[FloorPlan] = None,
                 nav: Optional[List[NavigationField]] = None,
                 initial_positions: Optional[np.ndarray] = None, verbose: bool = True):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.sim.seed)
        self.floor = floor or FloorPlan(cfg.map, cfg.exits, base_dir)
        if nav is None:
            if verbose:
                print("Building route fields (cached after the first run)...")
            nav = build_navigation(self.floor, cfg.exits, cfg.nav, base_dir, verbose)
        self.nav = nav
        self.signs = [ExitSign(i, ex, self.floor, cfg.sensor, cfg.gate, cfg.nav.evac_depth,
                               rng=np.random.default_rng([cfg.sim.seed, 1000 + i]))
                      for i, ex in enumerate(cfg.exits)]

        self.rooms = None
        if initial_positions is None:
            initial_positions = self._spawn_in_rooms(cfg.spawn.n_agents)
        n = len(initial_positions)
        v0 = self.rng.uniform(cfg.ped.v0_min, cfg.ped.v0_max, n)
        self.crowd = Crowd(initial_positions, v0, cfg.ped, cfg.force, self.rng, self.nav)
        self.n_total = n

        self.dt = cfg.sim.dt
        self.sensor_every = max(1, int(round(cfg.sensor.period / self.dt)))
        self.frame_every = max(1, int(round(cfg.sim.frame_period / self.dt)))
        self.step_idx = 0
        self.t = 0.0
        self.evac_time = np.full(n, np.nan)       # by agent id
        self.evac_exit = np.full(n, -1)
        self.recorder = Recorder(self)
        self._sense_and_control()                  # initial reading at t = 0
        self.recorder.record()

    # ------------------------------------------------------------------ spawning
    def _spawn_in_rooms(self, n: int) -> np.ndarray:
        """Each agent picks a random room, then a random free spot inside it."""
        sp, fl = self.cfg.spawn, self.floor
        self.rooms = fl.detect_rooms(sp.door_close_radius_px)
        n_rooms = int(self.rooms.max())
        if n_rooms == 0:
            raise RuntimeError("No spawn rooms detected; check SpawnConfig.door_close_radius_px")
        r = self.cfg.ped.radius
        ok = (fl.sdf_open >= r + sp.min_wall_gap) & (self.rooms > 0)
        if sp.x_min_m is not None:
            ok[:, :int(sp.x_min_m / fl.s)] = False
        if sp.x_max_m is not None:
            ok[:, int(sp.x_max_m / fl.s) + 1:] = False
        # Must be able to reach at least one exit.
        reach = np.zeros_like(ok)
        ys, xs = np.nonzero(ok)
        pts = np.stack([(xs + 0.5) * fl.s, (ys + 0.5) * fl.s], axis=1)
        dmin = np.min([nf.distance(pts) for nf in self.nav], axis=0)
        reach[ys[np.isfinite(dmin)], xs[np.isfinite(dmin)]] = True
        ok &= reach
        cand = {k: np.flatnonzero(ok & (self.rooms == k)) for k in range(1, n_rooms + 1)}
        rooms_avail = [k for k in cand if len(cand[k])]

        placed = np.zeros((0, 2))
        min_d = 2 * r + sp.min_agent_gap
        for _ in range(n):
            for attempt in range(1000):
                k = rooms_avail[self.rng.integers(len(rooms_avail))]
                flat = cand[k][self.rng.integers(len(cand[k]))]
                py, px = divmod(int(flat), fl.W)
                p = np.array([(px + self.rng.random()) * fl.s, (py + self.rng.random()) * fl.s])
                if len(placed) == 0 or np.min(np.linalg.norm(placed - p, axis=1)) >= min_d:
                    placed = np.vstack([placed, p])
                    break
            else:
                raise RuntimeError("Could not place all agents; rooms are too full")
        return placed

    # ------------------------------------------------------------------ sensing & control
    def _sense_and_control(self) -> None:
        for s in self.signs:
            count = s.count(self.crowd.pos)
            s.update(self.t, count)

    def _mark_evacuated(self, mask: np.ndarray, exit_idx: int) -> None:
        ids = self.crowd.remove(mask)
        self.evac_time[ids] = self.t
        self.evac_exit[ids] = exit_idx

    # ------------------------------------------------------------------ main loop
    @property
    def done(self) -> bool:
        return len(self.crowd) == 0 or self.t >= self.cfg.sim.t_max - 1e-9

    def step(self) -> None:
        if self.step_idx % self.sensor_every == 0 and self.step_idx > 0:
            self._sense_and_control()
        self.crowd.step(self.floor, self.dt)
        for s in self.signs:
            ev = s.evacuated(self.crowd.pos)
            if ev.any():
                self._mark_evacuated(ev, s.idx)
        self.step_idx += 1
        self.t = self.step_idx * self.dt
        if self.step_idx % self.frame_every == 0:
            self.recorder.record()

    def advance_frame(self) -> None:
        """Advance one rendered frame (frame_period of sim time)."""
        for _ in range(self.frame_every):
            if self.done:
                return
            self.step()

    def run(self, progress: bool = True) -> dict:
        last_print = -10.0
        while not self.done:
            self.advance_frame()
            if progress and self.t - last_print >= 10.0:
                last_print = self.t
                print(f"  t={self.t:6.1f}s  remaining={len(self.crowd):4d}  "
                      f"counts={[s.last_count for s in self.signs]}  "
                      f"closed={[int(s.closed) for s in self.signs]}", flush=True)
        return self.recorder.summary()

    def save_results(self, out_dir: str) -> None:
        os.makedirs(out_dir, exist_ok=True)
        self.recorder.to_csv(os.path.join(out_dir, "timeseries.csv"))
        self.recorder.save_summary(os.path.join(out_dir, "summary.json"))
        self.recorder.plot(os.path.join(out_dir, "timeseries.png"))

