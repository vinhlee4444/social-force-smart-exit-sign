"""
Simulation: owns the clock and calls every component in the right order.

One step (dt):
  1. every sensor period: each ExitSign counts the people it sees and updates its gate;
     a gate change repaints the door as wall/floor. Every frame, agents that reach a closed
     gate notice it, react after a delay and re-route (no instant global knowledge)
  2. look up each agent's desired direction from the route field of its target exit
  3. Social Force Model forces + integration (Crowd.step)
  4. remove agents that reached an exit room
  5. every frame period: record metrics
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
        self.crowd = Crowd(initial_positions, v0, cfg.ped, cfg.force, self.rng)
        self.n_total = n

        self.dt = cfg.sim.dt
        self.sensor_every = max(1, int(round(cfg.sensor.period / self.dt)))
        self.frame_every = max(1, int(round(cfg.sim.frame_period / self.dt)))
        self.step_idx = 0
        self.t = 0.0
        self.evac_time = np.full(n, np.nan)       # by agent id
        self.evac_exit = np.full(n, -1)
        self.assign_targets()
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

    # ------------------------------------------------------------------ decisions
    def assign_targets(self) -> None:
        """Every agent heads for the nearest OPEN exit (walking distance).
        If every exit is closed, agents keep their current target."""
        c = self.crowd
        if len(c) == 0:
            return
        D = np.stack([nf.distance(c.pos) for nf in self.nav], axis=1)
        closed = np.array([s.closed for s in self.signs])
        D[:, closed] = np.inf
        has_open = np.isfinite(D).any(axis=1)
        c.target[has_open] = np.argmin(D[has_open], axis=1)

    def _sense_and_control(self) -> None:
        for s in self.signs:
            count = s.count(self.crowd.pos)
            # Hub rule: never close the gate if that would leave too few passable exits.
            others = sum(1 for o in self.signs if o is not s and o.passable)
            can_close = others >= self.cfg.gate.min_open_exits
            if s.update(self.t, count, can_close):
                if self.cfg.gate.mode == "block":
                    if s.closed:
                        # People already inside the door gap are let through, not crushed.
                        through = s.in_doorway(self.crowd.pos)
                        if through.any():
                            self._mark_evacuated(through, s.idx)
                    self.floor.set_gate(s.idx, s.closed)
                # mode "divert": the door itself is never shut, only the guidance changes

    def _update_awareness(self) -> None:
        """People are NOT told about closed gates. An agent heading for a closed gate only
        notices it once it is within `notice_distance` (walking), then needs a reaction delay,
        then picks the nearest exit that is open. Agents far away keep walking towards it."""
        g, c = self.cfg.gate, self.crowd
        if not g.reroute_on_close or len(c) == 0:
            return
        closed = np.array([s.closed for s in self.signs])
        if not closed.any():
            c.aware_at[:] = np.nan
            return
        d_tgt = np.empty(len(c))
        for k in np.unique(c.target):
            m = c.target == k
            d_tgt[m] = self.nav[k].distance(c.pos[m])
        blocked = closed[c.target]
        if g.mode == "divert":                      # people already in the queue do not turn back
            blocked &= d_tgt > g.commit_distance
        c.aware_at[~blocked] = np.nan
        new = blocked & (d_tgt <= g.notice_distance) & np.isnan(c.aware_at)
        if new.any():
            c.aware_at[new] = self.t + self.rng.uniform(g.reaction_delay_min, g.reaction_delay_max,
                                                        int(new.sum()))
        with np.errstate(invalid="ignore"):
            act = blocked & (c.aware_at <= self.t + 1e-9)
        if act.any() and not closed.all():
            D = np.stack([nf.distance(c.pos[act]) for nf in self.nav], axis=1)
            cur_D = D[np.arange(len(D)), c.target[act]]          # distance to the exit they are heading to
            D[:, closed] = np.inf
            ok = np.isfinite(D).any(axis=1)
            idx_all = np.flatnonzero(act)
            if g.mode == "divert" and g.cost_based_divert:
                # time = walk + queue wait; switch only if clearly faster than staying
                wait = np.array([s.estimated_wait_s(g.flow_per_m_s) for s in self.signs])
                v = np.maximum(c.v0[act], 0.3)[:, None]
                cost = D / v + wait[None, :]
                best = np.argmin(cost, axis=1)
                cost_best = cost[np.arange(len(cost)), best]
                cost_cur = cur_D / v[:, 0] + wait[c.target[act]]
                go = ok & (cost_best + g.switch_margin_s < cost_cur)
                stay = ok & ~go
                c.target[idx_all[go]] = best[go]
                c.aware_at[idx_all[go]] = np.nan
                c.aware_at[idx_all[stay]] = self.t + g.recheck_s   # look again a bit later
            else:
                idx = idx_all[ok]
                c.target[idx] = np.argmin(D[ok], axis=1)
                c.aware_at[idx] = np.nan

    def _desired_directions(self) -> np.ndarray:
        c = self.crowd
        e = np.zeros((len(c), 2))
        for k in np.unique(c.target):
            m = c.target == k
            e[m] = self.nav[k].direction(c.pos[m])
        return e

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
        if self.step_idx % self.frame_every == 0:
            self._update_awareness()
        self.crowd.step(self._desired_directions(), self.floor, self.dt)
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

