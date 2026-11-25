"""Door-mounted signs receive current camera states over a simulated network."""
from __future__ import annotations

import numpy as np

from .route_advice import CoveredRoutes, RouteCompatible, before_forward_branch


class DoorNetwork:
    """Atomically deliver one batch of door states from a completed sensor tick."""

    def __init__(self):
        self.sample_time = -np.inf
        self.states = ()

    def publish(self, t, states):
        if t < self.sample_time:
            raise ValueError("Door reports must be delivered in sensor time order")
        self.states = tuple(states)
        self.sample_time = t

    def recommendations(self, source, t):
        if t < self.sample_time or not self.states or self.states[source] != "RED":
            return ()
        return tuple(k for k, state in enumerate(self.states) if state == "GREEN" and k != source)


class CongestionGuidance:
    def __init__(self, sim):
        self.sim = sim
        self.cfg = sim.cfg.guidance
        self.network = DoorNetwork()
        self.route_policy = CoveredRoutes(sim) if self.cfg.policy == "covered_routes" else None
        self.compatibility = (RouteCompatible(sim.floor, self.cfg)
                              if self.cfg.policy == "route_compatible" else None)
        self.rng = np.random.default_rng([sim.cfg.sim.seed, 2010])
        self.next_check = np.zeros(sim.n_total)
        self.react_at = np.full(sim.n_total, np.inf)
        self.pending_exit = np.full(sim.n_total, -1, dtype=int) if self.compatibility is not None else None
        self.last_switch = np.full(sim.n_total, -np.inf)
        self.committed = np.zeros(sim.n_total, dtype=bool)
        self.events = []
        self.decisions = []

    def _readable(self, origin, positions):
        nearby = np.linalg.norm(positions - origin, axis=1) <= self.cfg.read_distance
        nearby[nearby] = self.sim.floor.visible_from(origin, positions[nearby])
        return nearby

    def publish(self, t, states):
        self.network.publish(t, states)
        if self.route_policy is not None:
            self.route_policy.publish(self.network, t)

    def recommendations(self, source, t):
        options = self.network.recommendations(source, t)
        if options and self.route_policy is not None:
            return self.route_policy.broadcasts.get(source, ((), "no_valid_green_report"))
        if self.compatibility is not None:
            return self.compatibility.displayed(source, options, t), "no_issued_route"
        return options, "no_valid_green_report"

    def update(self, t, track_commit=False):
        sim, cfg = self.sim, self.cfg
        c = sim.crowd
        if track_commit:
            for k, sign in enumerate(sim.signs):
                rows = np.flatnonzero(c.target == k)
                close = sign.distance_to_door(c.pos[rows]) <= cfg.commit_distance
                self.committed[c.ids[rows[close]]] = True
        states = self.network.states
        if not states or t < self.network.sample_time:
            return
        pending = np.isfinite(self.react_at[c.ids])
        clock = np.where(pending, self.react_at[c.ids], self.next_check[c.ids])
        red = np.asarray(states)[c.target] == "RED"
        ready = np.flatnonzero((clock <= t + 1e-9) & (red | pending))
        # Advance the existing decision clock even while the sign is green.
        self.next_check[c.ids[(clock <= t + 1e-9) & ~pending]] = t + cfg.decision_recheck_period
        if not len(ready):
            return

        targets = c.target[ready].copy()
        ids = c.ids[ready]
        self.next_check[ids] = t + cfg.decision_recheck_period
        self.react_at[ids] = np.inf
        readable = np.zeros(len(ready), dtype=bool)
        for source in np.unique(targets):
            group = targets == source
            sign = sim.signs[source]
            positions = c.pos[ready[group]]
            readable[group] = self._readable(sign.camera, positions)
            close = sign.distance_to_door(positions) <= cfg.commit_distance
            self.committed[ids[group][close]] = True

        for row, agent, source, can_read in zip(ready, ids, targets, readable):
            destination = None
            reacting = pending[row]
            if self.compatibility is None:
                options, unavailable = self.recommendations(source, t)
            else:
                options = self.network.recommendations(source, t)
                unavailable = "no_valid_green_report"
                if reacting:
                    options = tuple(k for k in options if k == self.pending_exit[agent])
                    unavailable = "issued_destination_no_longer_green"
                self.pending_exit[agent] = -1
            if not red[row]:
                reason = "target_no_longer_red"
            elif not can_read:
                reason = "source_sign_unreadable"
            elif self.committed[agent]:
                reason = "committed"
            elif t - self.last_switch[agent] < cfg.switch_cooldown:
                reason = "cooldown"
            elif not options:
                reason = unavailable
            else:
                pos = c.pos[row:row + 1]
                current = sim.nav[source].distance(pos)[0]
                lengths = [(sim.nav[k].distance(pos)[0], k) for k in options]
                choices = [(length, k) for length, k in lengths if np.isfinite([current, length]).all()
                           and length <= current + cfg.max_extra_distance]
                reason = "no_route_within_detour"
                routes = {}
                if choices and (cfg.policy == "forward_branch" or self.compatibility is not None):
                    choices = [(length, k) for length, k in choices
                               if length - current <= cfg.branch_extra_ratio * current]
                    reason = "branch_detour_budget"
                    if choices:
                        heading = c.vel[row] if np.linalg.norm(c.vel[row]) >= 0.3 else sim.nav[source].direction(pos)[0]
                        if self.compatibility is None:
                            choices = [(length, k) for length, k in choices
                                       if before_forward_branch(sim.nav[source], sim.nav[k], pos[0], heading, cfg)]
                            reason = "no_forward_branch"
                        else:
                            choices, reason, routes = self._compatible_choices(source, choices, pos[0], heading)
                if choices:
                    _, candidate = min(choices)
                    destination = int(candidate)
                    if self.compatibility is not None:
                        self.compatibility.issue(source, destination, routes[candidate], t)
                    if reacting:
                        c.target[row] = destination
                        self.last_switch[agent] = t
                        self.events.append((t, int(agent), int(source), destination))
                        reason = "switched"
                    elif self.rng.random() < cfg.switch_probability:
                        self.react_at[agent] = t + self.rng.uniform(*cfg.reaction_delay)
                        if self.pending_exit is not None:
                            self.pending_exit[agent] = destination
                        reason = "reaction_scheduled"
                    else:
                        reason = "probability_declined"
            self.decisions.append({
                "t_s": t, "id": int(agent), "source": f"E{source + 1}",
                "source_readable": bool(can_read), "report_t_s": self.network.sample_time,
                "old_exit": f"E{source + 1}",
                "new_exit": f"E{destination + 1}" if reason == "switched" else None,
                "recommended_exit": f"E{destination + 1}" if destination is not None else None,
                "phase": "reaction" if reacting else "decision", "reason": reason,
            })

    def _compatible_choices(self, source, choices, point, heading):
        allowed, routes = [], {}
        reason = None
        for length, candidate in sorted(choices):
            route, rejected = self.compatibility.check(self.sim.nav[source], self.sim.nav[candidate], point, heading)
            if route is None:
                if reason is None:
                    reason = rejected
            else:
                allowed.append((length, candidate))
                routes[candidate] = route
        return allowed, reason, routes
