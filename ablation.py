"""Headless comparison with the same initial state and head-on motion in all arms."""
from __future__ import annotations

import copy
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np

from scenarios import BASE, SCENARIOS, initial_positions, spawn_layout
from sfm.config import Config
from sfm.simulation import Simulation


def track_distance(crowd):
    distance = np.zeros(len(crowd))
    step = crowd.step

    def measure(floor, dt, desired=None):
        before = crowd.pos.copy()
        step(floor, dt, desired)
        distance[crowd.ids] += np.linalg.norm(crowd.pos - before, axis=1)

    crowd.step = measure
    return distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", choices=SCENARIOS, default="350_70-30")
    parser.add_argument("--new-policy", choices=("covered_routes", "forward_branch", "route_compatible"),
                        default="covered_routes")
    parser.add_argument("--verify-current-video", action="store_true",
                        help="require the old ON arm to reproduce the audit paired with the current MP4")
    args = parser.parse_args()
    case = args.only
    cfg = Config()
    cfg.sim.t_max = 600
    cfg.gate.close_above, cfg.gate.reopen_at_most = 0.7, 0.4
    geometry = Simulation(cfg, BASE, initial_positions=np.empty((0, 2)), verbose=False)
    masks = [s.mask for s in geometry.signs]
    layout, bounds = spawn_layout(geometry)
    quota = SCENARIOS[case]
    if any(required > len(side) for required, side in zip(quota, layout)):
        raise SystemExit(f"{geometry.floor.image_name}: quota {quota} exceeds verified slots "
                         f"{tuple(map(len, layout))}; upper bounds {bounds}")
    positions = initial_positions(layout, quota, cfg.sim.seed)
    summaries, times, moved, distances = {}, {}, {}, {}
    folder = Path(BASE) / "results/stair_study" / case / "seed0"
    reference = None
    if args.verify_current_video:
        audit_bytes = (folder / "guidance_audit.json").read_bytes()
        reference = json.loads(audit_bytes)
        video = folder / "OFF_vs_ON.mp4"
        if (hashlib.sha256(video.read_bytes()).hexdigest() != reference["video_sha256"]
                or geometry.floor.image_sha256 != reference["map_sha256"]
                or reference["profile"] != "experimental"):
            raise RuntimeError("Current video, map or profile does not match its guidance audit")
    initial_hash = None
    for label, policy in (("off", "off"), ("old", "congestion_guidance"), ("new", args.new_policy)):
        arm = copy.deepcopy(cfg)
        arm.guidance.policy = policy
        arm.gate.enabled = label != "off"
        sim = Simulation(arm, BASE, geometry.floor, geometry.nav, positions,
                         verbose=False, camera_masks=masks)
        digest = sim.initial_state_hash()
        if initial_hash is None:
            initial_hash = digest
        if digest != initial_hash:
            raise RuntimeError(f"Initial-state mismatch in {label}")
        print(f"{label} init SHA-256 {digest}", flush=True)
        distance = track_distance(sim.crowd)
        s = sim.run(progress=True)
        distances[label] = distance
        times[label] = sim.evac_time.copy()
        events = [] if sim.guidance is None else sim.guidance.events
        moved[label] = np.isin(np.arange(sim.n_total), [e[1] for e in events])
        summaries[label] = s
        s["unique_switched"] = int(moved[label].sum())
        if sim.guidance is not None:
            s["readable_decision_reasons"] = dict(Counter(
                d["reason"] for d in sim.guidance.decisions if d["source_readable"]))
        if sim.guidance is not None and sim.guidance.route_policy is not None:
            s["approach_coverage"] = sim.guidance.route_policy.coverage
            s["unobserved_decisions"] = sum(d["reason"] == "approach_unobserved"
                                           for d in sim.guidance.decisions)
        if label == "old" and reference is not None:
            if digest != reference["initial_sha256"] or sim.guidance.decisions != reference["decisions"]:
                raise RuntimeError("Old ON did not reproduce the current video audit")
            print(f"Old ON reproduced all {len(sim.guidance.decisions)} audited decisions", flush=True)
        s["mean_distance_m"] = float(distance.mean())
        print(label, {k: s[k] for k in ("status", "T90_s", "T100_s", "remaining",
                                        "restricted_mean_s", "switches", "unique_switched")}, flush=True)

    limited = {label: np.where(np.isfinite(values), values, cfg.sim.t_max)
               for label, values in times.items()}
    cohorts = {}
    for label in ("old", "new"):
        delta = limited[label] - limited["off"]
        extra = distances[label] - distances["off"]
        groups = {"switched": moved[label], "kept_target": ~moved[label]}
        if label == "new":
            groups["switched_under_old"] = moved["old"]
        cohorts[label] = {}
        for group, mask in groups.items():
            subset = delta[mask]
            added = extra[mask]
            cohorts[label][group] = {
                "n": len(subset),
                "mean_delta_s": float(subset.mean()) if len(subset) else None,
                "median_delta_s": float(np.median(subset)) if len(subset) else None,
                "slower": int((subset > 1e-8).sum()),
                "faster": int((subset < -1e-8).sum()),
                "equal": int((np.abs(subset) <= 1e-8).sum()),
                "mean_delta_distance_m": float(added.mean()) if len(added) else None,
                "median_delta_distance_m": float(np.median(added)) if len(added) else None,
            }
    folder /= {"forward_branch": "branch_ablation", "route_compatible": "route_compatible_ablation"}.get(
        args.new_policy, "ablation")
    folder.mkdir(parents=True, exist_ok=True)
    if reference is not None:
        (folder / "reference_guidance_audit.json").write_bytes(audit_bytes)
    (folder / "summary.json").write_text(json.dumps({
        "map_name": geometry.floor.image_name, "map_sha256": geometry.floor.image_sha256,
        "initial_sha256": initial_hash, "new_policy": args.new_policy,
        "seed": cfg.sim.seed, "quota": quota, "physics_dt_s": cfg.sim.dt,
        "policies": {"off": "off", "old": "congestion_guidance", "new": args.new_policy},
        "thresholds_people_m2": [cfg.gate.close_above, cfg.gate.reopen_at_most],
        "reference_video_sha256": None if reference is None else reference["video_sha256"],
        "reference_audit_sha256": None if reference is None else hashlib.sha256(audit_bytes).hexdigest(),
        "runs": summaries, "cohorts_vs_off": cohorts,
    }, indent=2), encoding="utf-8")
    with (folder / f"paired_{len(positions)}.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(("id", "off_s", "old_s", "new_s", "old_switched", "new_switched",
                         "old_minus_off_restricted_s", "new_minus_off_restricted_s",
                         "off_m", "old_m", "new_m", "old_minus_off_m", "new_minus_off_m"))
        for i in range(len(positions)):
            writer.writerow([i, *[times[k][i] if np.isfinite(times[k][i]) else "" for k in times],
                             int(moved["old"][i]), int(moved["new"][i]),
                             round(limited["old"][i] - limited["off"][i], 2),
                             round(limited["new"][i] - limited["off"][i], 2),
                             *[round(distances[label][i], 4) for label in ("off", "old", "new")],
                             *[round(distances[label][i] - distances["off"][i], 4) for label in ("old", "new")]])
    print(cohorts)
    print(folder)


if __name__ == "__main__":
    main()
