"""Yielding behavior for a stalled group at a single-file room doorway."""
from __future__ import annotations

import numpy as np


class RoomFlow:
    def __init__(self, floor, rooms, cfg, radius, n):
        self.floor = floor
        self.rooms = rooms
        self.cfg = cfg
        self.doors = floor.single_file_room_doors(rooms, radius)
        self.still_since = {}
        self.leaders = {}
        self.tried = {}
        self.override = np.zeros(n, dtype=bool)
        self.yield_direction = np.zeros((n, 2))

    def update(self, t, crowd):
        self.override.fill(False)
        self.yield_direction.fill(0)
        x, y = self.floor.to_px(crowd.pos)
        room_ids = self.rooms[y, x]
        speed = np.linalg.norm(crowd.vel, axis=1)
        for room, (door, outward) in self.doors.items():
            rows = np.flatnonzero(room_ids == room)
            if len(rows) < 2:
                self.still_since.pop(room, None)
                self.leaders.pop(room, None)
                self.tried.pop(room, None)
                continue
            ids = crowd.ids[rows]
            leader = self.leaders.get(room)
            if (speed[rows] >= self.cfg.stall_speed).any():
                self.still_since[room] = t
            stalled = t - self.still_since.setdefault(room, t) >= self.cfg.stall_duration - 1e-9
            if leader is None and not stalled:
                continue
            leader_present = leader in ids
            if not leader_present or stalled:
                length = np.full(len(rows), np.inf)
                for target in np.unique(crowd.target[rows]):
                    group = crowd.target[rows] == target
                    length[group] = crowd.nav[target].distance(crowd.pos[rows[group]])
                if not np.isfinite(length).any():
                    continue
                if not leader_present:
                    tried = self.tried[room] = set()
                else:
                    tried = self.tried[room]
                    # If the front stalls again, give the opening to another
                    # nearby person before asking the same leader to try again.
                    front = length <= length.min() + 2 * crowd.radius[rows]
                    candidates = front & ~np.isin(ids, list(tried))
                    if not candidates.any():
                        tried.clear()
                        candidates = front & (ids != leader)
                    if candidates.any():
                        length[~candidates] = np.inf
                leader = self.leaders[room] = ids[np.argmin(length)]
                tried.add(leader)
                self.still_since[room] = t

            followers = rows[ids != leader]
            self.override[crowd.ids[followers]] = True
            offset = crowd.pos[followers] - door
            # Clear the central lane by stepping toward the jamb sides. People
            # farther back yield in place, removing forward drive from the tail.
            in_mouth = np.linalg.norm(offset, axis=1) < 4 * crowd.radius[followers]
            side_step = followers[in_mouth]
            tangent = np.array([-outward[1], outward[0]])
            directions = np.sign(offset[in_mouth] @ tangent)[:, None] * tangent
            if len(side_step):
                check = crowd.pos[side_step] + directions * crowd.radius[side_step, None]
                clear = self.floor.wall_distance(check) >= crowd.radius[side_step]
                self.yield_direction[crowd.ids[side_step[clear]]] = directions[clear]

            row = rows[ids == leader][0]
            radius = crowd.radius[row]
            approach = crowd.pos[row] - door
            if np.linalg.norm(approach) < 2 * radius:
                # The exit field can begin the turn down the hall too early.
                # Keep the leader normal to the jamb while a full body length
                # straight ahead is clear.
                distances = np.linspace(0, 2 * radius, 21)
                ahead = crowd.pos[row] + distances[:, None] * outward
                if (self.floor.wall_distance(ahead) >= radius).all():
                    self.override[leader] = True
                    self.yield_direction[leader] = outward

    def directions(self, crowd, desired):
        overridden = self.override[crowd.ids]
        desired[overridden] = self.yield_direction[crowd.ids[overridden]]
        return desired
