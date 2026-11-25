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

from typing import TYPE_CHECKING, List, Optional

import numpy as np

from .config import ForceConfig, PedestrianConfig
from .floorplan import FloorPlan
from .navigation import NavigationField

if TYPE_CHECKING:
    from .exit_sign import ExitSign


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
        self.contact = np.zeros(n)                                 # N, smoothed compression force felt (0.3 s EMA)
        self.raw_contact = np.zeros(n)                             # N, instantaneous compression force at current step
        self._pairs_i = self._pairs_j = None
        self._pos_at_build: Optional[np.ndarray] = None
        if self.nav is not None and n > 0:
            self.assign_nearest_targets()

    def __len__(self) -> int:
        return len(self.pos)

    # ------------------------------------------------------------------ routing
    def assign_nearest_targets(self) -> None:
        """Every agent initially heads for the nearest exit (walking distance)."""
        if self.nav is None or len(self) == 0:
            return
        D = np.stack([nf.distance(self.pos) for nf in self.nav], axis=1)
        self.target = np.argmin(D, axis=1)

    @staticmethod
    def _can_see_sign(pos: np.ndarray, sign_pos: np.ndarray, floor: FloorPlan) -> np.ndarray:
        """True for each agent in `pos` that has an unobstructed line of sight to `sign_pos`."""
        n = len(pos)
        if n == 0:
            return np.zeros(0, dtype=bool)
        free = getattr(floor, "see_through", floor.base_free)
        s = floor.s
        px, py = pos[:, 0] / s, pos[:, 1] / s
        cx, cy = sign_pos[0] / s, sign_pos[1] / s
        dist_px = np.hypot(px - cx, py - cy)
        n_samp = max(2, int(np.ceil(np.max(dist_px) / 2.0)))
        ok = np.ones(n, dtype=bool)
        for step in range(1, n_samp):
            a = step / n_samp
            sx = np.clip((cx + a * (px - cx)).astype(np.int64), 0, floor.W - 1)
            sy = np.clip((cy + a * (py - cy)).astype(np.int64), 0, floor.H - 1)
            # Ignore wall pixels within ~6 px (0.3 m) of the agent body itself (e.g. squeezing past a door jamb)
            ok &= free[sy, sx] | ((1.0 - a) * dist_px < 6.0)
        return ok

    def update_targets(self, signs: List["ExitSign"], floor: FloorPlan,
                       commit_radius: float = 5.0) -> None:
        """Update each agent's target exit based on door proximity and visible exit sign LEDs:
        1. If within `commit_radius` (m) of the door: exit through that door despite the LED signal.
        2. If the nearest door is GREEN: run to the nearest door.
        3. If the nearest door is RED: run to the nearest visible GREEN door.
        """
        n = len(self)
        if n == 0 or self.nav is None or len(signs) == 0:
            return
        D = np.stack([nf.distance(self.pos) for nf in self.nav], axis=1)
        nearest = np.argmin(D, axis=1)
        closed = np.array([s.closed for s in signs], dtype=bool)
        if not closed.any():
            self.target = nearest
            return

        # Distance from each agent to each door center (and walking distance along the floor)
        door_centers = np.array([s.door_center for s in signs])
        d_euclid = np.linalg.norm(self.pos[:, None, :] - door_centers[None, :, :], axis=2)
        # An agent is within commit_radius of a door if within commit_radius walking distance
        # (or Euclidean distance with line-of-sight so it cannot trigger through classroom walls)
        d_cur_walk = D[np.arange(n), self.target]
        d_near_walk = D[np.arange(n), nearest]
        d_cur_euc = d_euclid[np.arange(n), self.target]
        d_near_euc = d_euclid[np.arange(n), nearest]

        commit_cur = (d_cur_euc <= commit_radius) & (d_cur_walk <= commit_radius + 2.0)
        commit_near = (~commit_cur) & (d_near_euc <= commit_radius) & (d_near_walk <= commit_radius + 2.0)
        self.target[commit_near] = nearest[commit_near]

        undecided = ~(commit_cur | commit_near)
        if not undecided.any():
            return
        idx_u = np.flatnonzero(undecided)
        near_u = nearest[idx_u]

        # If nearest door is GREEN, head to the nearest door
        near_green = ~closed[near_u]
        self.target[idx_u[near_green]] = near_u[near_green]

        # If nearest door is RED, head to the nearest visible GREEN door
        idx_red = idx_u[~near_green]
        if len(idx_red) == 0 or closed.all():
            return
        pos_r = self.pos[idx_red]
        vis_green = np.zeros((len(idx_red), len(signs)), dtype=bool)
        for k, sg in enumerate(signs):
            if not closed[k]:
                vis_green[:, k] = self._can_see_sign(pos_r, sg.camera, floor)

        D_red = D[idx_red].copy()
        D_red[~vis_green] = np.inf
        has_vis_green = np.isfinite(D_red).any(axis=1)
        if has_vis_green.any():
            self.target[idx_red[has_vis_green]] = np.argmin(D_red[has_vis_green], axis=1)

        # If no GREEN door is visible yet (e.g. agent is still inside a classroom):
        # keep current target if it is GREEN; otherwise head toward nearest exit to reach the corridor
        no_vis = idx_red[~has_vis_green]
        if len(no_vis):
            cur_closed = closed[self.target[no_vis]]
            self.target[no_vis[cur_closed]] = nearest[no_vis[cur_closed]]

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
        for name in ("pos", "vel", "v0", "radius", "mass", "ids", "target", "contact", "raw_contact"):
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
            self.raw_contact = contact
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
            rel_v = self.vel[j] - self.vel[i]
            dvt = np.einsum("ij,ij->i", rel_v, tij)
            dvn = np.maximum(np.einsum("ij,ij->i", rel_v, nij), 0.0)  # approaching normal speed
            damp_n = np.where(g > 0.0, 2000.0 * dvn, 0.0)             # inelastic normal collision damping
            mag_n = fc.A * np.exp(overlap / fc.B) + fc.k * g + damp_n
            fric_c = np.minimum(fc.kappa * g, 3000.0)        # prevent explicit-Euler tangential overshoot
            f = mag_n[:, None] * nij + (fric_c * dvt)[:, None] * tij
            F[:, 0] += np.bincount(i, f[:, 0], n) - np.bincount(j, f[:, 0], n)
            F[:, 1] += np.bincount(i, f[:, 1], n) - np.bincount(j, f[:, 1], n)
            contact += np.bincount(i, fc.k * g, n) + np.bincount(j, fc.k * g, n)

        # 3) Wall force from the signed distance field.
        dw = floor.wall_distance(self.pos)
        nw = floor.wall_normal(self.pos)                     # points away from the wall
        tw = np.stack([-nw[:, 1], nw[:, 0]], axis=1)
        overlap_w = self.radius - dw
        gw = np.maximum(overlap_w, 0.0)
        v_into_wall = np.maximum(-np.einsum("ij,ij->i", self.vel, nw), 0.0)
        damp_w = np.where(gw > 0.0, 2000.0 * v_into_wall, 0.0)
        mag_w = fc.A_wall * np.exp(overlap_w / fc.B_wall) + fc.k * gw + damp_w
        vt = np.einsum("ij,ij->i", self.vel, tw)
        fric_w = np.minimum(fc.kappa * gw, 3000.0)
        F += mag_w[:, None] * nw - (fric_w * vt)[:, None] * tw
        contact += fc.k * gw

        # 4) Optional random fluctuation.
        if fc.noise_std > 0:
            F += self.rng.normal(0.0, fc.noise_std, size=F.shape)

        self.raw_contact = contact
        return F

    def step(self, floor: FloorPlan, dt: float,
             desired_dir: Optional[np.ndarray] = None) -> None:
        """Advance one time step with semi-implicit Euler."""
        if len(self) == 0:
            return
        if desired_dir is None:
            desired_dir = self.desired_directions()
        F = self.compute_forces(desired_dir, floor)
        # Smooth per-agent contact force over a 0.3 s rolling window so transient
        # 0.01 s shoulder bumps do not flicker the crush-danger counter.
        alpha = dt / (0.3 + dt)
        self.contact = (1.0 - alpha) * self.contact + alpha * self.raw_contact
        self.vel += F / self.mass[:, None] * dt
        speed = np.linalg.norm(self.vel, axis=1)
        vmax = self.ped.v_max_factor * self.v0
        too_fast = speed > vmax
        self.vel[too_fast] *= (vmax[too_fast] / speed[too_fast])[:, None]
        self.pos += self.vel * dt

