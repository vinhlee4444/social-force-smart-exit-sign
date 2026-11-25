"""
Crowd: all pedestrians, stored as NumPy arrays (one row per agent).

Social Force Model (Helbing, Farkas & Vicsek, Nature 2000):

    m dv/dt = m (v0 e - v) / tau                                   driving force
            + sum_j [A exp((r_ij - d_ij)/B) + k g(r_ij - d_ij)] n_ij  social + body force
                    + kappa g(r_ij - d_ij) dv_t t_ij               sliding friction
            + same three terms for the nearest wall                wall force

    g(x) = max(x, 0)  -> body & friction forces only act when bodies touch.
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from .config import ForceConfig, PedestrianConfig
from .floorplan import FloorPlan
from .navigation import NavigationField


def pair_force_magnitudes(force: ForceConfig, overlap, tangential_speed):
    """Social repulsion, body compression and sliding friction for a pedestrian pair."""
    compression = np.maximum(overlap, 0.0)
    return (force.A * np.exp(overlap / force.B), force.k * compression,
            force.kappa * compression * tangential_speed)


class Crowd:
    def __init__(self, pos: np.ndarray, v0: np.ndarray, ped: PedestrianConfig,
                 force: ForceConfig, rng: np.random.Generator,
                 nav: Optional[List[NavigationField]] = None):
        n = len(pos)
        self.ped = ped
        self.fc = force
        self.rng = rng
        self.nav = nav
        self.pos = np.asarray(pos, float).reshape(n, 2).copy()     # m
        self.vel = np.zeros((n, 2))                                # m/s
        self.v0 = np.asarray(v0, float).copy()                     # desired speed, m/s
        self.radius = np.full(n, ped.radius)                       # m
        self.mass = np.full(n, ped.mass)                           # kg
        self.ids = np.arange(n)                                    # stable id per agent
        self.target = np.zeros(n, dtype=np.int64)                  # index of the nearest exit each agent heads for
        self.contact = np.zeros(n)                                 # N, compression force felt (crush proxy)
        self._pairs_i = self._pairs_j = None
        self._pos_at_build: Optional[np.ndarray] = None
        self.counterflow_rooms: Optional[np.ndarray] = None
        if self.nav is not None and n > 0:
            self.assign_nearest_targets()

    def __len__(self) -> int:
        return len(self.pos)

    # ------------------------------------------------------------------ routing
    def assign_nearest_targets(self) -> None:
        """Every agent heads for the nearest exit (walking distance), regardless of gate state."""
        if self.nav is None or len(self) == 0:
            return
        D = np.stack([nf.distance(self.pos) for nf in self.nav], axis=1)
        if not np.isfinite(D).any(axis=1).all():
            raise ValueError("At least one agent has no body-clear route to a destination")
        self.target = np.argmin(D, axis=1)

    def desired_directions(self) -> np.ndarray:
        """Unit desired walking direction for every agent from its target exit's route field."""
        e = np.zeros((len(self), 2))
        if self.nav is None or len(self) == 0:
            return e
        for k in np.unique(self.target):
            m = self.target == k
            e[m] = self.nav[k].direction(self.pos[m])
        return e

    # ------------------------------------------------------------------ bookkeeping
    def remove(self, mask: np.ndarray) -> np.ndarray:
        """Remove agents where mask is True. Returns their ids."""
        gone = self.ids[mask]
        keep = ~mask
        for name in ("pos", "vel", "v0", "radius", "mass", "ids", "target", "contact"):
            setattr(self, name, getattr(self, name)[keep])
        self._pos_at_build = None                                  # force neighbour-list rebuild
        return gone

    def _neighbour_pairs(self):
        """Verlet list: all pairs closer than cutoff + skin, rebuilt only when needed."""
        n = len(self)
        rebuild = (self._pos_at_build is None or len(self._pos_at_build) != n or
                   np.max(np.linalg.norm(self.pos - self._pos_at_build, axis=1), initial=0.0)
                   > 0.5 * self.fc.skin)
        if rebuild:
            if n < 2:
                self._pairs_i = self._pairs_j = np.zeros(0, dtype=np.int64)
            else:
                d = self.pos[:, None, :] - self.pos[None, :, :]
                d2 = np.einsum("ijk,ijk->ij", d, d)
                rc = self.fc.cutoff + self.fc.skin
                i, j = np.nonzero(np.triu(d2 < rc * rc, k=1))
                self._pairs_i, self._pairs_j = i, j
            self._pos_at_build = self.pos.copy()
        return self._pairs_i, self._pairs_j

    # ------------------------------------------------------------------ physics
    def _avoid_head_on(self, desired: np.ndarray, floor: FloorPlan) -> np.ndarray:
        """Choose a body-clear passing side before opposing walkers meet."""
        i, j = self._neighbour_pairs()
        if not len(i):
            return desired
        delta = self.pos[j] - self.pos[i]
        opposition = np.einsum("ij,ij->i", desired[i], desired[j]) < -0.7
        if not opposition.any():
            return desired
        i, j, delta = i[opposition], j[opposition], delta[opposition]
        right = np.column_stack((-desired[:, 1], desired[:, 0]))
        ahead_i = np.einsum("ij,ij->i", delta, desired[i])
        ahead_j = -np.einsum("ij,ij->i", delta, desired[j])
        lateral = np.abs(np.einsum("ij,ij->i", delta, right[i]))
        near = ((ahead_i > 0) & (ahead_j > 0)
                & (lateral < self.radius[i] + self.radius[j] + self.fc.B)
                & (np.linalg.norm(delta, axis=1) <= self.fc.cutoff))
        i, j, delta = i[near], j[near], delta[near]
        if len(i) and self.counterflow_rooms is not None:
            x, y = floor.to_px(self.pos)
            corridor = self.counterflow_rooms[y, x] == 0
            in_corridor = corridor[i] & corridor[j]
            i, j, delta = i[in_corridor], j[in_corridor], delta[in_corridor]
        if not len(i):
            return desired
        # Discard interactions across partitions.
        fractions = np.linspace(0, 1, int(np.ceil(2 * self.fc.cutoff / floor.s)) + 1)
        points = self.pos[i, None] + delta[:, None] * fractions[None, :, None]
        x, y = floor.to_px(points.reshape(-1, 2))
        clear = floor.base_free[y, x].reshape(len(i), -1).all(axis=1)
        i, j = i[clear], j[clear]
        rows = np.unique(np.r_[i, j])
        if not len(rows):
            return desired
        # Trade some forward progress for a lateral step with the same desired
        # speed and relaxation time. This tie-break also works at zero speed.
        sidestep = right[rows] + 0.35 * desired[rows]
        sidestep /= np.linalg.norm(sidestep, axis=1, keepdims=True)
        radius = self.radius[rows]
        ahead = self.pos[rows, None] + sidestep[:, None] * (radius[:, None] * np.linspace(0, 1, 11))[..., None]
        clear = (floor.wall_distance(ahead.reshape(-1, 2)).reshape(len(rows), -1)
                 >= radius[:, None]).all(axis=1)
        direction = desired.copy()
        direction[rows[clear]] = sidestep[clear]
        return direction

    def compute_forces(self, desired_dir: np.ndarray, floor: FloorPlan) -> np.ndarray:
        """Total force on every agent (N x 2), in newtons."""
        fc, n = self.fc, len(self)
        F = np.zeros((n, 2))
        contact = np.zeros(n)
        if n == 0:
            self.contact = contact
            return F

        # 1) Driving force: relax towards the desired velocity.
        F += self.mass[:, None] * (self.v0[:, None] * desired_dir - self.vel) / self.ped.tau

        # 2) Agent-agent forces (computed once per pair, applied with opposite signs).
        i, j = self._neighbour_pairs()
        if len(i):
            diff = self.pos[i] - self.pos[j]
            dist = np.maximum(np.linalg.norm(diff, axis=1), 1e-6)
            nij = diff / dist[:, None]                       # points from j to i
            tij = np.stack([-nij[:, 1], nij[:, 0]], axis=1)
            overlap = self.radius[i] + self.radius[j] - dist
            dvt = np.einsum("ij,ij->i", self.vel[j] - self.vel[i], tij)
            social, body, friction = pair_force_magnitudes(fc, overlap, dvt)
            f = (social + body)[:, None] * nij + friction[:, None] * tij
            F[:, 0] += np.bincount(i, f[:, 0], n) - np.bincount(j, f[:, 0], n)
            F[:, 1] += np.bincount(i, f[:, 1], n) - np.bincount(j, f[:, 1], n)
            contact += np.bincount(i, body, n) + np.bincount(j, body, n)

        # 3) Wall force from the signed distance field.
        dw = floor.wall_distance(self.pos)
        nw = floor.wall_normal(self.pos)                     # points away from the wall
        tw = np.stack([-nw[:, 1], nw[:, 0]], axis=1)
        overlap_w = self.radius - dw
        gw = np.maximum(overlap_w, 0.0)
        social_wall = fc.A_wall * np.exp(overlap_w / fc.B_wall)
        wall_force = social_wall[:, None] * nw
        # The closest jamb can point straight back out of a clear doorway.
        # Preserve transverse repulsion and contact, but allow driving along
        # a forward segment that has room for the whole body.
        along = np.einsum("ij,ij->i", nw, desired_dir)
        check = (along < 0) & (dw >= self.radius) & (dw < self.radius + 4 * fc.B_wall)
        if check.any():
            radius = self.radius[check]
            direction = desired_dir[check]
            fractions = np.linspace(0, 2, 21)[1:]
            ahead = (self.pos[check, None] +
                     direction[:, None] * (radius[:, None] * fractions)[..., None])
            clearance = floor.wall_distance(ahead.reshape(-1, 2)).reshape(len(radius), -1)
            through = (clearance >= radius[:, None]).all(axis=1)
            idx = np.flatnonzero(check)[through]
            wall_force[idx] -= (social_wall[idx] * along[idx])[:, None] * desired_dir[idx]
        vt = np.einsum("ij,ij->i", self.vel, tw)
        F += wall_force + (fc.k * gw)[:, None] * nw - (fc.kappa * gw * vt)[:, None] * tw
        contact += fc.k * gw

        # 4) Optional random fluctuation.
        if fc.noise_std > 0:
            F += self.rng.normal(0.0, fc.noise_std, size=F.shape)

        self.contact = contact
        return F

    def step(self, floor: FloorPlan, dt: float,
             desired_dir: Optional[np.ndarray] = None) -> None:
        """Advance one time step with semi-implicit Euler."""
        if len(self) == 0:
            return
        if desired_dir is None:
            desired_dir = self.desired_directions()
        if self.ped.avoid_head_on:
            desired_dir = self._avoid_head_on(desired_dir, floor)
        F = self.compute_forces(desired_dir, floor)
        self.vel += F / self.mass[:, None] * dt
        speed = np.linalg.norm(self.vel, axis=1)
        vmax = self.ped.v_max_factor * self.v0
        too_fast = speed > vmax
        self.vel[too_fast] *= (vmax[too_fast] / speed[too_fast])[:, None]
        self.pos += self.vel * dt

