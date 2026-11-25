"""Seed-0 OFF/ON study on the repository floor plan."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from datetime import datetime
from itertools import combinations
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np

from sfm.config import Config
from sfm.floorplan import FloorPlan
from sfm.simulation import Simulation

BASE = Path(__file__).resolve().parent
SCENARIOS = {
    "100_20-80": (20, 80),
    "100_70-30": (70, 30),
    "350_20-80": (70, 280),
    "350_70-30": (245, 105),
    "600_20-80": (200, 800),
    "600_70-30": (700, 300),
}


def check_door_visibility(sim):
    """Sample body-clear floor and count positions with both door signs readable."""
    floor, cfg = sim.floor, sim.cfg.guidance
    stride = 2
    y, x = np.nonzero(floor.sdf_open[::stride, ::stride] >= sim.cfg.ped.radius)
    positions = (np.column_stack((x, y)) * stride + 0.5) * floor.s
    ranges = [np.linalg.norm(positions - sign.camera, axis=1) <= cfg.read_distance
              for sign in sim.signs]
    step = stride * floor.s
    print(f"Door sight lines sampled every {step:g} m over body-clear floor:", flush=True)
    for a, b in combinations(sim.signs, 2):
        candidates = positions[ranges[a.idx] & ranges[b.idx]]
        shared = []
        for start in range(0, len(candidates), 2048):
            pts = candidates[start:start + 2048]
            pts = pts[floor.visible_from(a.camera, pts)]
            shared.append(pts[floor.visible_from(b.camera, pts)])
        visible = np.concatenate(shared) if shared else np.empty((0, 2))
        da, db = [sim.nav[s.idx].distance(visible) for s in (a, b)]
        reachable = np.isfinite(da) & np.isfinite(db)
        switch_ab = reachable & (a.distance_to_door(visible) > cfg.commit_distance)
        switch_ab &= db <= da + cfg.max_extra_distance
        switch_ba = reachable & (b.distance_to_door(visible) > cfg.commit_distance)
        switch_ba &= da <= db + cfg.max_extra_distance
        print(f"  E{a.idx + 1}/E{b.idx + 1}: within both ranges={len(candidates)}, "
              f"readable together={len(visible)}; "
              f"eligible A->B={switch_ab.sum()}, B->A={switch_ba.sum()}", flush=True)


def spawn_layout(sim):
    """Search deterministic room packings and bound capacity with disjoint disks."""
    floor, sp = sim.floor, sim.cfg.spawn
    rooms = floor.detect_rooms(sp.door_close_radius_px)
    split = floor.split_x_m
    gap = 2 * sim.cfg.ped.radius + sp.min_agent_gap
    clearance = sim.cfg.ped.radius + sp.min_wall_gap
    best = [[], []]
    # Optimize packing phases locally so a common grid origin does not waste room space.
    for room in range(1, int(rooms.max()) + 1):
        ry, rx = np.nonzero(rooms == room)
        lo = np.array([rx.min(), ry.min()]) * floor.s
        hi = (np.array([rx.max(), ry.max()]) + 1) * floor.s
        for side in (0, 1):
            room_lo, room_hi = lo.copy(), hi.copy()
            if side == 0:
                room_hi[0] = min(room_hi[0], split)
            else:
                room_lo[0] = max(room_lo[0], split)
            if (room_hi <= room_lo).any():
                continue
            chosen = np.empty((0, 2))
            for transpose in (False, True):
                dims = (room_hi - room_lo)[::-1] if transpose else room_hi - room_lo
                for hexagonal in (True, False):
                    spacing = gap + 1e-8
                    height = spacing * (np.sqrt(3) / 2 if hexagonal else 1)
                    ii, jj = np.meshgrid(np.arange(np.ceil(dims[0] / spacing)),
                                         np.arange(np.ceil(dims[1] / height)))
                    grid = np.column_stack(((ii.ravel() + (0.5 * (jj.ravel() % 2)
                                            if hexagonal else 0)) * spacing, jj.ravel() * height))
                    for dx in np.linspace(0, spacing, 8, endpoint=False):
                        for dy in np.linspace(0, height, 8, endpoint=False):
                            offsets = grid + [dx, dy]
                            if transpose:
                                offsets = offsets[:, ::-1]
                            pts = room_lo + offsets[(offsets < room_hi - room_lo).all(axis=1)]
                            x, y = floor.to_px(pts)
                            pts = pts[(rooms[y, x] == room) & (floor.wall_distance(pts) >= clearance)]
                            if len(pts) <= len(chosen):
                                continue
                            if best[0] or best[1]:
                                already = np.asarray(best[0] + best[1])
                                nearby = ((already >= room_lo - gap) & (already <= room_hi + gap)).all(axis=1)
                                pts = pts[(np.linalg.norm(pts[:, None] - already[nearby], axis=2)
                                           >= gap).all(axis=1)]
                            if len(pts) > len(chosen):
                                reachable = np.isfinite([nav.distance(pts) for nav in sim.nav]).any(axis=0)
                                pts = pts[reachable]
                                if len(pts) > len(chosen):
                                    chosen = pts
            best[side].extend(chosen)

    bounds = packing_upper_bounds(floor, rooms, clearance, gap)
    return [np.asarray(side).reshape(-1, 2) for side in best], bounds


def packing_upper_bounds(floor, rooms, clearance, gap):
    """Sum Oler's packing bound over convex room-side enclosures."""
    # Bilinear clearance anywhere in a pixel cannot exceed the maximum
    # of its 3x3 neighborhood. Enclose entire candidate pixels in the hull.
    split = floor.split_x_m
    max_sdf = cv2.dilate(floor.sdf_open, np.ones((3, 3), np.uint8))
    corners = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])
    bounds = [0, 0]
    for room in range(1, int(rooms.max()) + 1):
        y, x = np.nonzero((rooms == room) & (max_sdf >= clearance))
        for side, inside in enumerate((x * floor.s < split, (x + 1) * floor.s > split)):
            centers = np.column_stack((x[inside], y[inside])).astype(np.int32)
            if not len(centers):
                continue
            hull = cv2.convexHull(centers).reshape(-1, 2)
            enclosure = cv2.convexHull((hull[:, None] + corners).reshape(-1, 2).astype(np.int32))
            area = cv2.contourArea(enclosure) * floor.s ** 2
            edges = np.diff(enclosure[:, 0], axis=0, append=enclosure[:1, 0])
            perimeter = np.linalg.norm(edges, axis=1).sum() * floor.s
            # https://doi.org/10.4153/CMB-1961-018-7
            bound = 2 * area / (np.sqrt(3) * gap ** 2) + perimeter / (2 * gap) + 1
            bounds[side] += int(bound + 1e-8)
    return bounds


def initial_positions(layout, quota, seed):
    rng = np.random.default_rng(seed)
    points = np.concatenate([side[rng.choice(len(side), n, replace=False)]
                             for side, n in zip(layout, quota)])
    return points[rng.permutation(len(points))]


def export_camera_masks(sim, directory):
    directory.mkdir(parents=True, exist_ok=True)
    for sign in sim.signs:
        # Registered to the original raster. Blue pixels are exactly the ROI.
        image = np.repeat(np.where(sim.floor.base_free, 242, 45).astype(np.uint8)[..., None], 3, axis=2)
        image[sign.mask] = (239, 133, 25)
        path = directory / f"E{sign.idx + 1}_mask.png"
        if not cv2.imwrite(str(path), image):
            raise OSError(f"Could not write {path}")
        print(path.resolve(), flush=True)


def track_guidance(sim):
    """Record distinct readers at camera ticks."""
    guide = sim.guidance
    seen = [set() for _ in sim.signs]
    target_seen = [set() for _ in sim.signs]
    update = guide.update

    def record_readers(t, track_commit=False):
        if sim.step_idx % sim.sensor_every == 0:
            c = sim.crowd
            for sign, state in zip(sim.signs, guide.network.states):
                if state == "RED":
                    reads = guide._readable(sign.camera, c.pos)
                    seen[sign.idx].update(c.ids[reads])
                    target_seen[sign.idx].update(c.ids[reads & (c.target == sign.idx)])
        update(t, track_commit)

    guide.update = record_readers
    return seen, target_seen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", nargs="+", choices=SCENARIOS)
    parser.add_argument("--map", type=Path, help="map path, relative to the repository or absolute")
    parser.add_argument("--expect-map-sha256", help="reject a map whose source bytes do not match this hash")
    parser.add_argument("--rebuild-nav", action="store_true", help="bypass and rebuild the navigation cache")
    parser.add_argument("--preview", action="store_true", help="save the paired first frame without writing MP4")
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="verify a temporary 6-agent video")
    parser.add_argument("--threshold-profile", choices=("reference", "experimental"), default="reference")
    parser.add_argument("--guidance-policy", choices=("congestion_guidance", "covered_routes", "forward_branch",
                                                       "route_compatible"),
                        default="congestion_guidance")
    parser.add_argument("--headless", action="store_true", help="collect metrics without encoding a video")
    parser.add_argument("--camera-masks", type=Path, help="export four registered ROI images to this directory")
    args = parser.parse_args()
    if args.preview and (args.headless or args.smoke or args.preflight):
        parser.error("--preview cannot be combined with --headless, --smoke or --preflight")

    cfg = Config()
    configured_map = (BASE / cfg.map.image_path).resolve()
    if args.map is not None:
        selected_map = (BASE / args.map).resolve()
        cfg.map.image_path = str(selected_map)
    cfg.nav.rebuild_cache = args.rebuild_nav
    cfg.sim.seed, cfg.sim.t_max = 0, 600.0
    cfg.guidance.policy = args.guidance_policy
    if args.threshold_profile == "experimental":
        cfg.gate.close_above, cfg.gate.reopen_at_most = 0.7, 0.4
    print(f"Threshold profile {args.threshold_profile}: RED > {cfg.gate.close_above:g}, "
          f"GREEN <= {cfg.gate.reopen_at_most:g} people/m2", flush=True)
    floor = FloorPlan(cfg.map, cfg.exits, BASE)
    print(f"Runner opened: {floor.image_path}\nMap SHA-256: {floor.image_sha256}", flush=True)
    if args.expect_map_sha256 and floor.image_sha256 != args.expect_map_sha256.lower():
        parser.error("--expect-map-sha256 does not match the loaded image")
    if floor.image_path != configured_map:
        configured_hash = hashlib.sha256(configured_map.read_bytes()).hexdigest()
        if floor.image_sha256 != configured_hash:
            parser.error("--map changes the configured raster. Update its door/camera geometry first.")
    geometry = Simulation(cfg, BASE, floor=floor, initial_positions=np.empty((0, 2)))
    masks = [s.mask for s in geometry.signs]
    for sign in geometry.signs:
        print(f"{sign.name}: area={sign.visible_area_m2:.3f} m2; blind={sign.min_range:.3f} m; "
              f"range={sign.effective_range:.3f}/{cfg.sensor.radius:.1f} m", flush=True)
    if args.camera_masks is not None:
        export_camera_masks(geometry, args.camera_masks)
    layout, bounds = spawn_layout(geometry)
    print(f"Split x={geometry.floor.split_x_m:.2f} m; verified slots L/R: "
          f"{[len(side) for side in layout]}; geometric upper bounds L/R: {bounds}", flush=True)
    feasible = {}
    for key, quota in SCENARIOS.items():
        if all(n <= len(side) for n, side in zip(quota, layout)):
            feasible[key] = quota
            print(f"{key}: PASS {quota}, n={sum(quota)}", flush=True)
        else:
            errors = []
            for label, n, side, bound in zip(("left", "right"), quota, layout, bounds):
                if n > len(side):
                    detail = f"geometric upper bound {bound}" if n > bound else "no valid layout found"
                    errors.append(f"{label} needs {n}, verified slots {len(side)} ({detail})")
            print(f"{key}: configuration error: {', '.join(errors)}", flush=True)
    if args.preflight:
        check_door_visibility(geometry)
        return

    from sfm.render import comparison_preview, save_comparison

    off_cfg = copy.deepcopy(cfg)
    off_cfg.gate.enabled = False
    off_cfg.guidance.policy = "off"
    chosen = {k: feasible[k] for k in (args.only or SCENARIOS) if k in feasible}
    skipped = [k for k in (args.only or ()) if k not in feasible]
    for key in skipped:
        output = BASE / "results/stair_study" / key / "seed0/OFF_vs_ON.mp4"
        print(f"{key}: not published; existing file preserved if present: {output}", flush=True)
    if args.smoke:
        cfg.sim.t_max = off_cfg.sim.t_max = 3.0
        chosen = {"smoke": (3, 3)}
    for key, quota in chosen.items():
        pos = initial_positions(layout, quota, cfg.sim.seed)
        off, on = [Simulation(c, BASE, geometry.floor, geometry.nav, pos, verbose=False,
                              camera_masks=masks) for c in (off_cfg, cfg)]
        state_hash = off.initial_state_hash()
        if state_hash != on.initial_state_hash():
            raise RuntimeError(f"{key}: OFF/ON initial state mismatch")
        print(f"{key}: n={len(pos)} quota={quota} initial SHA-256={state_hash}", flush=True)
        if args.smoke:
            with TemporaryDirectory() as temp:
                save_comparison(off, on, Path(temp) / "OFF_vs_ON.mp4", quota)
            print("Smoke MP4 encoded and fully decoded", flush=True)
            continue
        output = BASE / "results/stair_study" / key / "seed0/OFF_vs_ON.mp4"
        if args.preview:
            preview = output.with_name("preview.png")
            preview.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(preview), comparison_preview(off, on, quota)):
                raise OSError(f"Could not write preview {preview}")
            print(f"Pipeline preview: {preview}", flush=True)
            continue
        readers = track_guidance(on)
        first_frame = output.with_name("first_frame.png")
        summaries = ((off.run(progress=False), on.run(progress=False)) if args.headless
                     else save_comparison(off, on, output, quota, first_frame_path=first_frame))
        decisions = on.guidance.decisions
        eligible = [{d["id"] for d in decisions if d["source"] == f"E{i+1}"
                     and d["recommended_exit"] is not None} for i in range(len(on.signs))]
        readers = (*readers, eligible)
        for name, summary in zip(("OFF", "ON"), summaries):
            cameras = [(e["max_raw_value"], e["max_value"], e["red_entries"], e["total_red_s"])
                       for e in summary["exits"]]
            print(f"{key} {name}: {summary['status']} T90={summary['T90_s']} T100={summary['T100_s']} "
                  f"remaining={summary['remaining']} mean={summary['restricted_mean_s']:.2f} "
                  f"switches={summary['switches']} cameras E1-E4 (raw, smoothed, red entries, red s)={cameras} "
                  f"progress={summary['progress']}", flush=True)
        for idx, values in enumerate(zip(*readers), 1):
            switches = sum(event[2] == idx - 1 for event in on.guidance.events)
            print(f"E{idx} ON unique RED readers / target RED readers / eligible: "
                  f"{tuple(map(len, values))}; switches={switches}", flush=True)
        print(f"ON unique RED readers / target RED readers / eligible: "
              f"{tuple(len(set().union(*values)) for values in readers)}; "
              f"unique switched={len({event[1] for event in on.guidance.events})}", flush=True)
        if not args.headless:
            audit = {
                "profile": args.threshold_profile, "initial_sha256": state_hash,
                "policy": cfg.guidance.policy,
                "map_name": geometry.floor.image_name, "map_sha256": geometry.floor.image_sha256,
                "map_path": str(geometry.floor.image_path),
                "video_path": str(output), "first_frame_path": str(first_frame),
                "video_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                "video_updated_at": datetime.fromtimestamp(output.stat().st_mtime).astimezone().isoformat(
                    timespec="seconds"),
                "meters_per_pixel": geometry.floor.s, "spawn_split_x_m": geometry.floor.split_x_m,
                "source": "door-mounted display, camera states relayed by simulated network",
                "readers_sample_period_s": cfg.sensor.period,
                "doors": [], "decisions": decisions,
            }
            for i, values in enumerate(zip(*readers), 1):
                seen, targets, eligible_ids = (sorted(map(int, ids)) for ids in values)
                source = f"E{i}"
                read_at_decision = {d["id"] for d in decisions if d["source"] == source
                                    and d["source_readable"] and d["reason"] != "target_no_longer_red"}
                audit["doors"].append({
                    "source": source, "red_reader_ids": seen, "target_red_reader_ids": targets,
                    "eligible_ids": eligible_ids,
                    "no_decision_while_reading_ids": sorted(set(targets) - read_at_decision),
                })
            output.with_name("guidance_audit.json").write_text(json.dumps(audit, indent=2), encoding="utf-8")
            print(f"Published: {output}\nUpdated: {audit['video_updated_at']}\n"
                  f"Video SHA-256: {audit['video_sha256']}\nDecoded first frame: {first_frame}", flush=True)
    return 1 if skipped else 0


if __name__ == "__main__":
    raise SystemExit(main())
