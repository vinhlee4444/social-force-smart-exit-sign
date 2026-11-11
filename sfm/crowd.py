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

from typing import Optional

import numpy as np

from .config import ForceConfig, PedestrianConfig
from .floorplan import FloorPlan


class Crowd:
    def __init__(self, pos: np.ndarray, v0: np.ndarray, ped: PedestrianConfig,
                 force: ForceConfig, rng: np.random.Generator):
        n = len(pos)
        self.ped = ped
        self.fc = force
        self.rng = rng
        self.pos = np.asarray(pos, float).reshape(n, 2).copy()     # m
        self.vel = np.zeros((n, 2))                                # m/s
        self.v0 = np.asarray(v0, float).copy()                     # desired speed, m/s
        self.radius = np.full(n, ped.radius)                       # m
        self.mass = np.full(n, ped.mass)                           # kg
        self.ids = np.arange(n)                                    # stable id per agent
        self.target = np.zeros(n, dtype=np.int64)                  # index of the exit each agent heads for
        self.contact = np.zeros(n)                                 # N, compression force felt (crush proxy)
        self.aware_at = np.full(n, np.nan)                         # s, when this agent decides to leave a closed exit
        self._pairs_i = self._pairs_j = None
        self._pos_at_build: Optional[np.ndarray] = None

    def __len__(self) -> int:
        return len(self.pos)

    # ------------------------------------------------------------------ bookkeeping
    def remove(self, mask: np.ndarray) -> np.ndarray:
        """Remove agents where mask is True. Returns their ids."""
        gone = self.ids[mask]
        keep = ~mask
        for name in ("pos", "vel", "v0", "radius", "mass", "ids", "target", "contact", "aware_at"):
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
            g = np.maximum(overlap, 0.0)
            dvt = np.einsum("ij,ij->i", self.vel[j] - self.vel[i], tij)
            mag_n = fc.A * np.exp(overlap / fc.B) + fc.k * g
            f = mag_n[:, None] * nij + (fc.kappa * g * dvt)[:, None] * tij
            F[:, 0] += np.bincount(i, f[:, 0], n) - np.bincount(j, f[:, 0], n)
            F[:, 1] += np.bincount(i, f[:, 1], n) - np.bincount(j, f[:, 1], n)
            contact += np.bincount(i, fc.k * g, n) + np.bincount(j, fc.k * g, n)

        # 3) Wall force from the signed distance field.
        dw = floor.wall_distance(self.pos)
        nw = floor.wall_normal(self.pos)                     # points away from the wall
        tw = np.stack([-nw[:, 1], nw[:, 0]], axis=1)
        overlap_w = self.radius - dw
        gw = np.maximum(overlap_w, 0.0)
        mag_w = fc.A_wall * np.exp(overlap_w / fc.B_wall) + fc.k * gw
        vt = np.einsum("ij,ij->i", self.vel, tw)
        F += mag_w[:, None] * nw - (fc.kappa * gw * vt)[:, None] * tw
        contact += fc.k * gw

        # 4) Optional random fluctuation.
        if fc.noise_std > 0:
            F += self.rng.normal(0.0, fc.noise_std, size=F.shape)

        self.contact = contact
        return F

    def step(self, desired_dir: np.ndarray, floor: FloorPlan, dt: float) -> None:
        """Advance one time step with semi-implicit Euler."""
        if len(self) == 0:
            return
        F = self.compute_forces(desired_dir, floor)
        self.vel += F / self.mass[:, None] * dt
        speed = np.linalg.norm(self.vel, axis=1)
        vmax = self.ped.v_max_factor * self.v0
        too_fast = speed > vmax
        self.vel[too_fast] *= (vmax[too_fast] / speed[too_fast])[:, None]
        self.pos += self.vel * dt

