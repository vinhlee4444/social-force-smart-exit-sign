"""Inspect the revised floor plan before running paired videos."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scenarios import BASE, spawn_layout
from sfm.config import Config
from sfm.crowd import Crowd
from sfm.simulation import Simulation


def measure_portal(floor, center, outward):
    """Find the smallest continuously open transverse raster span near a portal."""
    x, y = (center / floor.s).astype(int)
    across_y = bool(outward[0])
    plane = floor.base_free.T if across_y else floor.base_free
    row, col = (x, y) if across_y else (y, x)
    widths = []
    for line in plane[row - 3:row + 4]:
        if not line[col]:
            continue
        blocked = np.flatnonzero(~line)
        left, right = blocked[blocked < col], blocked[blocked > col]
        if len(left) and len(right):
            widths.append((right[0] - left[-1] - 1) * floor.s)
    return min(widths, default=None)


def room_replays(sim, rooms, slots):
    floor, cfg = sim.floor, sim.cfg
    x, y = floor.to_px(slots)
    labels = rooms[y, x]
    ids = np.arange(1, int(rooms.max()) + 1)
    starts, missing = {}, []
    for room in ids:
        candidates = slots[labels == room]
        if not len(candidates):
            missing.append(int(room))
            continue
        # Test a body-clear location far from any destination, inside this room.
        route_length = np.min([nav.distance(candidates) for nav in sim.nav], axis=0)
        starts[int(room)] = candidates[route_length.argmax()]
    starts_array = np.array(list(starts.values()))
    c = Crowd(starts_array, np.full(len(starts), cfg.ped.v0_min), cfg.ped, cfg.force,
              np.random.default_rng(cfg.sim.seed), sim.nav)
    # Vectorize independent single-person replays. No agent-agent interaction.
    c._neighbour_pairs = lambda: (np.empty(0, dtype=int), np.empty(0, dtype=int))
    targets, paths = c.target.copy(), [list([pt.copy()]) for pt in c.pos]
    arrivals = np.full(len(c), np.nan)
    min_wall = np.full(len(c), np.inf)
    for tick in range(round(90 / cfg.sim.dt)):
        c.step(floor, cfg.sim.dt)
        min_wall[c.ids] = np.minimum(min_wall[c.ids], floor.wall_distance(c.pos))
        if tick % 20 == 0:
            for i, point in zip(c.ids, c.pos):
                paths[i].append(point.copy())
        mask = np.logical_or.reduce([sign.evacuated(c.pos) for sign in sim.signs])
        if mask.any():
            for i, point in zip(c.ids[mask], c.pos[mask]):
                paths[i].append(point.copy())
            arrivals[c.remove(mask)] = (tick + 1) * cfg.sim.dt
        if not len(c):
            break
    rows = [dict(room=room, target=f"E{targets[i] + 1}", arrival_s=float(arrivals[i]) if np.isfinite(arrivals[i]) else None,
                 start=starts[room].tolist(), wall_clearance_min_m=round(float(min_wall[i]), 4))
            for i, room in enumerate(starts)]
    return rows, paths, missing


def corridor_replay(sim, name, center, transverse, n=1):
    floor, cfg = sim.floor, sim.cfg
    center, transverse = np.array(center), np.array(transverse)
    forward = np.array([transverse[1], -transverse[0]])
    positions = np.array([center + forward * sign * (1 + 1.45 * i)
                          for sign in (-1, 1) for i in range(n)])
    c = Crowd(positions, np.full(2 * n, 1.6), cfg.ped, cfg.force, np.random.default_rng(0))
    directions = np.array([forward] * n + [-forward] * n)
    clearance = np.inf
    for _ in range(round(15 / cfg.sim.dt)):
        c.step(floor, cfg.sim.dt, directions[c.ids])
        clearance = min(clearance, floor.wall_distance(c.pos).min())
        projected = (c.pos - center) @ forward
        passed = np.where(c.ids < n, projected > 2, projected < -2)
        if passed.any():
            c.remove(passed)
        if not len(c):
            break
    return dict(name=name, width_m=measure_portal(floor, center, forward),
                center=center.tolist(), people=2 * n, passed=2 * n - len(c), min_wall_clearance_m=float(clearance))


def main():
    sim = Simulation(Config(), BASE, initial_positions=np.empty((0, 2)), verbose=False)
    floor = sim.floor
    out = BASE / "results/map_check" / floor.image_sha256[:12]
    out.mkdir(parents=True, exist_ok=True)
    layout, bounds = spawn_layout(sim)
    slots = np.concatenate(layout)
    rooms = floor.detect_rooms(sim.cfg.spawn.door_close_radius_px)
    rows, paths, missing = room_replays(sim, rooms, slots)
    doors = floor.single_file_room_doors(rooms, sim.cfg.ped.radius)
    portal_widths = {room: measure_portal(floor, door, direction)
                     for room, (door, direction) in doors.items()}
    corridors = [
        corridor_replay(sim, "north hall", (42.5, 6.075), (0, 1)),
        corridor_replay(sim, "north hall", (42.5, 6.075), (0, 1), n=3),
        corridor_replay(sim, "central-west passage", (23.25, 17.5), (1, 0)),
        corridor_replay(sim, "central-west passage", (23.25, 17.5), (1, 0), n=3),
        corridor_replay(sim, "east passage", (50.375, 11.0), (1, 0)),
    ]

    w, h = floor.width_m, floor.height_m
    fig, ax = plt.subplots(figsize=(17, 10))
    ax.imshow(floor.base_free, cmap="Greys_r", extent=(0, w, h, 0), interpolation="nearest")
    palette = np.array([[0.29, 0.64, 0.95], [0.96, 0.51, 0.2], [0.52, 0.82, 0.42], [0.78, 0.50, 0.9]])
    overlays = np.zeros((*rooms.shape, 4))
    for sign, color in zip(sim.signs, palette):
        overlays[sign.mask] = (*color, 0.63)
    ax.imshow(overlays, extent=(0, w, h, 0), interpolation="nearest")
    for side, points in enumerate(layout):
        ax.scatter(*points.T, s=3, color=("#357d9e", "#b8493a")[side], alpha=0.46)
    ax.axvline(floor.split_x_m, linestyle="--", color="#834fb8", label=f"L/R x={floor.split_x_m:.2f} m")
    for row in rows:
        room = row["room"]
        yy, xx = np.where(rooms == room)
        label = (np.array([xx.mean(), yy.mean()]) + 0.5) * floor.s
        ax.text(*label, f"R{room}", ha="center", va="center", fontsize=8,
                bbox=dict(fc="white", ec="none", alpha=0.8))
        if room in doors:
            door, normal = doors[room]
            text = f"D{room} {portal_widths[room]:.2f} m" if portal_widths[room] is not None else f"D{room}"
            ax.text(*(door - 0.7 * normal), text, ha="center", fontsize=6.5,
                    bbox=dict(fc="#ffffde", ec="none", alpha=0.88))
    exits = []
    for sign, color in zip(sim.signs, palette):
        x0, y0, x1, y1 = sign.cfg.door
        width = floor.base_free[y0:y1 + 1, x0:x1 + 1].all(axis=0).sum() * floor.s
        ax.plot(*sign.camera, marker="^" if sign.facing[1] < 0 else "v", color=color,
                markeredgecolor="#171926")
        ax.quiver(*sign.camera, *(sign.facing * 1.2), color=color, angles="xy",
                  scale_units="xy", scale=1, width=.004)
        ax.text(*(sign.door_center + sign.inward * 1.65),
                f"E{sign.idx + 1} / {width:.2f} m\nROI {sign.visible_area_m2:.3f} m2",
                ha="center", va="center", fontsize=8,
                bbox=dict(fc="white", ec=color, alpha=0.92))
        exits.append(dict(door=f"E{sign.idx + 1}", width_m=width, camera=sign.camera.tolist(),
                          facing=sign.facing.tolist(), roi_m2=sign.visible_area_m2,
                          near_m=sign.min_range, far_m=sign.effective_range))
    for side in (0, 1):
        matching = [i for i, row in enumerate(rows) if row["arrival_s"] is not None
                    and int(row["start"][0] >= floor.split_x_m) == side
                    and int(row["target"][1:]) in ((2, 4) if side else (1, 3))]
        if matching:
            i = max(matching, key=lambda j: rows[j]["arrival_s"])
            path = np.array(paths[i])
            ax.plot(*path.T, color=("#087fbb", "#c84726")[side], lw=2.3,
                    label=f"Solo R{rows[i]['room']} to {rows[i]['target']}")
    ax.set(xlim=(0, w), ylim=(h, 0), aspect="equal", xlabel="x (m)", ylabel="y (m)")
    ax.set_title(f"{floor.image_name}  |  {floor.W} x {floor.H} px  |  {floor.s:g} m/px  |  SHA-256 {floor.image_sha256[:16]}")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.10), ncol=3, fontsize=9)
    fig.savefig(out / "geometry.png", dpi=140, bbox_inches="tight")
    plt.close(fig)
    result = dict(map=floor.image_name, sha256=floor.image_sha256, pixels=[floor.W, floor.H],
                  meters_per_pixel=floor.s, split_x_m=floor.split_x_m, slots=list(map(len, layout)),
                  bounds=bounds, exits=exits, room_portal_width_m=portal_widths, solo=rows,
                  rooms_without_valid_slots=missing, counterflow=corridors)
    (out / "geometry.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(out.resolve())
    print("Room solo failures:", [row for row in rows if row["arrival_s"] is None], "No slots:", missing)
    print("Counterflow:", corridors)


if __name__ == "__main__":
    main()
