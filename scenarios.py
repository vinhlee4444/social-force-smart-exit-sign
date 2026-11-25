"""
Benchmark runner for the 6 Floor-2 occupancy scenarios:

  1. scen1_sparse           : ~200 people across the building (sparse slot).
  2. scen2_normal           : 15% of classrooms empty (0), 15% low occupancy (2-5 people),
                              70% normal occupancy (70-100% of load: 21-30 small, 28-40 large).
  3. scen3_asymmetric_left  : ~80-85% of population in left half (scattered across empty, low,
                              and normal rooms); ~15-20% in right half.
  4. scen4_asymmetric_right : ~80-85% of population in right half (including the right hall
                              and right/central classrooms); ~15-20% in left half.
  5. scen5_asymmetric_top   : ~80-85% of population in top half (plus normal-scenario random
                              load in the right hall); ~15-20% in bottom half.
  6. scen6_full_house       : 100% of classrooms at normal/full occupancy (21-30 small, 28-40 large).

Each scenario is run across 5 seeds (0..4) both WITH smart exit signs and WITHOUT smart exit signs
(60 runs total -> 60 .csv files + 60 .json files), plus 1 .mp4 video (seed 0) per (scenario, gate state)
(12 .mp4 files total).
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics as st
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Dict, List, Tuple

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)

from sfm.config import Config                    # noqa: E402
from sfm.floorplan import FloorPlan              # noqa: E402
from sfm.navigation import NavigationField       # noqa: E402
from sfm.simulation import Simulation            # noqa: E402

SCENARIOS = {
    "scen1_sparse": dict(
        title="1 (sparse: ~200 people)",
    ),
    "scen2_normal": dict(
        title="2 (normal: 15% empty, 15% low, 70% normal load)",
    ),
    "scen3_asymmetric_left": dict(
        title="3 (asymmetric_left: 80-85% left half)",
    ),
    "scen4_asymmetric_right": dict(
        title="4 (asymmetric_right: 80-85% right half)",
    ),
    "scen5_asymmetric_top": dict(
        title="5 (asymmetric_top: 80-85% top half)",
    ),
    "scen6_full_house": dict(
        title="6 (full_house: 100% normal/full load)",
    ),
}

GATE_MODES = {
    "with_smart_exit": True,
    "without_smart_exit": False,
}


def classify_rooms(floor: FloorPlan, nav: List[NavigationField], cfg: Config):
    """Detect all rooms and classify non-restroom classrooms into 'small', 'large', or 'hall'."""
    rooms = floor.detect_rooms(cfg.spawn.door_close_radius_px)
    n_rooms = int(rooms.max())
    r = cfg.ped.radius
    ok = (floor.sdf_open >= r + 0.05) & (rooms > 0)
    reach = np.zeros_like(ok)
    ys, xs = np.nonzero(ok)
    pts = np.stack([(xs + 0.5) * floor.s, (ys + 0.5) * floor.s], axis=1)
    dmin = np.min([nf.distance(pts) for nf in nav], axis=0)
    reach[ys[np.isfinite(dmin)], xs[np.isfinite(dmin)]] = True
    ok &= reach

    room_info = []
    for k in range(1, n_rooms + 1):
        mask = rooms == k
        area_m2 = float(np.sum(mask)) * (floor.s ** 2)
        if area_m2 < 15.0:
            # The 4 small 12.6 m^2 rooms in the top-middle and bottom-middle are restrooms
            continue
        rys, rxs = np.nonzero(mask)
        cx_m = float(np.mean(rxs)) * floor.s
        cy_m = float(np.mean(rys)) * floor.s
        cand = np.flatnonzero(ok & mask)
        if len(cand) == 0:
            continue
        if area_m2 > 100.0:
            kind = "hall"
        elif area_m2 < 28.0:
            kind = "small"
        else:
            kind = "large"
        room_info.append(dict(k=k, kind=kind, area_m2=area_m2, cx_m=cx_m, cy_m=cy_m, cand=cand))
    return rooms, room_info


def sample_room_counts(room_info: List[Dict], scenario_key: str, rng: np.random.Generator) -> Dict[int, int]:
    """Assign how many people spawn in each classroom for a given scenario and seed."""
    n_cls = len(room_info)
    counts: Dict[int, int] = {}

    def normal_load(rm: Dict, high: bool = False) -> int:
        # 70% to 100% of capacity: 21..30 for small rooms (max 30), 28..40 for large rooms (max 40)
        if rm["kind"] == "small":
            return int(rng.integers(25 if high else 21, 31))
        return int(rng.integers(33 if high else 28, 41))

    if scenario_key == "scen1_sparse":
        # ~200 people across the building: 40% empty, 25% low (2-5 people), 35% moderate load (12-22 people)
        perm = rng.permutation(n_cls)
        n_empty = int(round(0.40 * n_cls))
        n_low = int(round(0.25 * n_cls))
        for idx_pos, r_idx in enumerate(perm):
            rm = room_info[int(r_idx)]
            if idx_pos < n_empty:
                counts[rm["k"]] = 0
            elif idx_pos < n_empty + n_low:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = int(rng.integers(14, 22))

    elif scenario_key == "scen2_normal":
        perm = rng.permutation(n_cls)
        n_empty = int(round(0.15 * n_cls))       # 15% empty (5 rooms)
        n_low = int(round(0.15 * n_cls))         # 15% low occupancy 2-5 people (4-5 rooms)
        for idx_pos, r_idx in enumerate(perm):
            rm = room_info[int(r_idx)]
            if idx_pos < n_empty:
                counts[rm["k"]] = 0
            elif idx_pos < n_empty + n_low:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = normal_load(rm)

    elif scenario_key == "scen3_asymmetric_left":
        # Left half (cx_m <= 33.0 m, 18 rooms): ~80-85% of total population, with some empty,
        # some low (2-5), and most at normal load.
        # Right half (cx_m > 33.0 m, 12 rooms): ~15-20% of total population, with a few normal,
        # some 1-3 people, and rest empty.
        left_rooms = [rm for rm in room_info if rm["cx_m"] <= 33.0]
        right_rooms = [rm for rm in room_info if rm["cx_m"] > 33.0]
        perm_l = rng.permutation(len(left_rooms))
        for idx_pos, r_idx in enumerate(perm_l):
            rm = left_rooms[int(r_idx)]
            if idx_pos < 2:
                counts[rm["k"]] = 0
            elif idx_pos < 4:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = normal_load(rm, high=True)
        perm_r = rng.permutation(len(right_rooms))
        for idx_pos, r_idx in enumerate(perm_r):
            rm = right_rooms[int(r_idx)]
            if idx_pos < 3:
                counts[rm["k"]] = normal_load(rm)
            elif idx_pos < 7:
                counts[rm["k"]] = int(rng.integers(1, 4))
            else:
                counts[rm["k"]] = 0

    elif scenario_key == "scen4_asymmetric_right":
        # Right half (cx_m >= 27.0 m, including right hall & right/central rooms): ~80-85% of population.
        # Left half (cx_m < 27.0 m): ~15-20% of population (a few normal rooms, some 1-3, rest empty).
        right_rooms = [rm for rm in room_info if rm["cx_m"] >= 27.0]
        left_rooms = [rm for rm in room_info if rm["cx_m"] < 27.0]
        perm_r = rng.permutation(len(right_rooms))
        for idx_pos, r_idx in enumerate(perm_r):
            rm = right_rooms[int(r_idx)]
            if rm["kind"] == "hall":
                # Large 340 m^2 right hall can hold multiple classes / study groups (80-110 students)
                counts[rm["k"]] = int(rng.integers(85, 110))
            elif idx_pos < 2:
                counts[rm["k"]] = 0
            elif idx_pos < 4:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = normal_load(rm, high=True)
        perm_l = rng.permutation(len(left_rooms))
        for idx_pos, r_idx in enumerate(perm_l):
            rm = left_rooms[int(r_idx)]
            if idx_pos < 3:
                counts[rm["k"]] = normal_load(rm)
            elif idx_pos < 6:
                counts[rm["k"]] = int(rng.integers(1, 4))
            else:
                counts[rm["k"]] = 0

    elif scenario_key == "scen5_asymmetric_top":
        # Top half (cy_m <= 17.5 m, excluding the right hall): crowded (~80-85% of classroom population).
        # Right hall (kind == "hall"): random normal-scenario load (15% empty, 15% low, 70% normal 28-40).
        # Bottom half (cy_m > 17.5 m, excluding right hall): ~15-20% (a few normal, some 1-3, rest empty).
        top_rooms = [rm for rm in room_info if rm["kind"] != "hall" and rm["cy_m"] <= 17.5]
        bot_rooms = [rm for rm in room_info if rm["kind"] != "hall" and rm["cy_m"] > 17.5]
        halls = [rm for rm in room_info if rm["kind"] == "hall"]

        for rm in halls:
            u = rng.random()
            if u < 0.15:
                counts[rm["k"]] = 0
            elif u < 0.30:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = normal_load(rm)

        perm_t = rng.permutation(len(top_rooms))
        for idx_pos, r_idx in enumerate(perm_t):
            rm = top_rooms[int(r_idx)]
            if idx_pos < 1:
                counts[rm["k"]] = 0
            elif idx_pos < 3:
                counts[rm["k"]] = int(rng.integers(2, 6))
            else:
                counts[rm["k"]] = normal_load(rm, high=True)

        perm_b = rng.permutation(len(bot_rooms))
        for idx_pos, r_idx in enumerate(perm_b):
            rm = bot_rooms[int(r_idx)]
            if idx_pos < 2:
                counts[rm["k"]] = normal_load(rm)
            elif idx_pos < 6:
                counts[rm["k"]] = int(rng.integers(1, 4))
            else:
                counts[rm["k"]] = 0

    elif scenario_key == "scen6_full_house":
        for rm in room_info:
            counts[rm["k"]] = normal_load(rm)
    else:
        raise ValueError(f"Unknown scenario '{scenario_key}'")

    return counts


def spawn_positions_for_scenario(cfg: Config, floor: FloorPlan, nav: List[NavigationField],
                                 scenario_key: str, seed: int) -> Tuple[np.ndarray, np.ndarray]:
    """Generate initial agent (x, y) positions for (scenario_key, seed)."""
    rng = np.random.default_rng([seed, 777])
    rooms, room_info = classify_rooms(floor, nav, cfg)
    counts = sample_room_counts(room_info, scenario_key, rng)

    r = cfg.ped.radius
    min_d = 2 * r + 0.02
    all_pts = []
    for rm in room_info:
        c_need = counts.get(rm["k"], 0)
        if c_need <= 0:
            continue
        cand = rm["cand"]
        placed = np.zeros((0, 2))
        for _ in range(c_need):
            for attempt in range(2500):
                flat = cand[rng.integers(len(cand))]
                py, px = divmod(int(flat), floor.W)
                p = np.array([(px + rng.random()) * floor.s, (py + rng.random()) * floor.s])
                if len(placed) == 0 or np.min(np.linalg.norm(placed - p, axis=1)) >= min_d:
                    placed = np.vstack([placed, p])
                    break
            else:
                raise RuntimeError(f"Room {rm['k']} ({rm['kind']}) could not fit {c_need} agents")
        all_pts.append(placed)

    pos = np.vstack(all_pts) if all_pts else np.zeros((0, 2))
    perm = rng.permutation(len(pos))
    return pos[perm], rooms


def run_single_sim(args) -> Dict:
    """Run one headless simulation and write its .csv and .json files."""
    sc_key, gate_key, seed, out_dir = args
    cfg = Config()
    cfg.sim.seed = seed
    cfg.gate.enabled = GATE_MODES[gate_key]

    sim_tmp = Simulation(cfg, BASE, initial_positions=np.zeros((1, 2)), verbose=False)
    pos, rooms = spawn_positions_for_scenario(cfg, sim_tmp.floor, sim_tmp.nav, sc_key, seed)
    cfg.spawn.n_agents = len(pos)

    sim = Simulation(cfg, BASE, floor=sim_tmp.floor, nav=sim_tmp.nav,
                     initial_positions=pos, verbose=False)
    sim.rooms = rooms
    s = sim.run(progress=False)

    os.makedirs(out_dir, exist_ok=True)
    stem = f"{sc_key}__{gate_key}__seed{seed}"
    csv_path = os.path.join(out_dir, f"{stem}.csv")
    json_path = os.path.join(out_dir, f"{stem}.json")
    sim.recorder.to_csv(csv_path)
    sim.recorder.save_summary(json_path)

    tet = s["T100_s"] if s["T100_s"] is not None else s["sim_time_s"]
    return dict(
        scenario=sc_key,
        gate_mode=gate_key,
        seed=seed,
        n_agents=s["n_agents"],
        evacuated=s["evacuated"],
        all_evacuated=s["all_evacuated"],
        TET=tet,
        AET=s["AET_s"],
        TPID=s["total_people_in_danger"],
        peak_danger=s["peak_agents_in_danger"],
        danger_ps=s["danger_person_seconds"],
        closures=sum(e["closures"] for e in s["exits"]),
        csv_file=csv_path,
        json_file=json_path,
    )


def render_single_video(args) -> str:
    """Render one .mp4 video for (sc_key, gate_key, seed)."""
    import matplotlib
    matplotlib.use("Agg")
    from sfm.render import Renderer

    sc_key, gate_key, seed, video_dir, fps, dpi = args
    cfg = Config()
    cfg.sim.seed = seed
    cfg.gate.enabled = GATE_MODES[gate_key]

    sim_tmp = Simulation(cfg, BASE, initial_positions=np.zeros((1, 2)), verbose=False)
    pos, rooms = spawn_positions_for_scenario(cfg, sim_tmp.floor, sim_tmp.nav, sc_key, seed)
    cfg.spawn.n_agents = len(pos)

    sim = Simulation(cfg, BASE, floor=sim_tmp.floor, nav=sim_tmp.nav,
                     initial_positions=pos, verbose=False)
    sim.rooms = rooms

    os.makedirs(video_dir, exist_ok=True)
    mp4_path = os.path.join(video_dir, f"{sc_key}__{gate_key}.mp4")
    Renderer(sim).save(mp4_path, fps=fps, dpi=dpi, progress=False)
    return mp4_path


def main() -> int:
    ap = argparse.ArgumentParser(description="Run 6 scenarios x 2 gate modes x 5 seeds + 12 MP4 videos")
    ap.add_argument("--seeds", type=int, default=5, help="number of seeds per scenario/mode (default: 5)")
    ap.add_argument("--out", default=os.path.join(BASE, "results", "benchmark"),
                    help="output folder for CSV, JSON, and MP4 files")
    ap.add_argument("--workers", type=int, default=min(6, os.cpu_count() or 4))
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--dpi", type=int, default=80)
    ap.add_argument("--skip-videos", action="store_true")
    args = ap.parse_args()

    runs_dir = os.path.join(args.out, "runs")
    video_dir = os.path.join(args.out, "videos")
    os.makedirs(runs_dir, exist_ok=True)
    os.makedirs(video_dir, exist_ok=True)

    Simulation(Config(), BASE, initial_positions=np.zeros((1, 2)), verbose=False)

    sim_tasks = [
        (sc_key, gate_key, seed, runs_dir)
        for sc_key in SCENARIOS
        for gate_key in GATE_MODES
        for seed in range(args.seeds)
    ]

    print(f"Running {len(sim_tasks)} simulations ({len(SCENARIOS)} scenarios x 2 gate states x {args.seeds} seeds)...")
    t0 = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        for i, r in enumerate(ex.map(run_single_sim, sim_tasks), 1):
            rows.append(r)
            print(f"  [{i:2d}/{len(sim_tasks)}] {r['scenario']:23s} | {r['gate_mode']:18s} | "
                  f"seed {r['seed']} (N={r['n_agents']:3d}) -> TET={r['TET']:5.2f}s, "
                  f"AET={r['AET']:5.2f}s, TPID={r['TPID']:3d}, closures={r['closures']}", flush=True)

    summary_csv = os.path.join(args.out, "all_runs_summary.csv")
    with open(summary_csv, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print("\n" + "=" * 102)
    print(f"{'Scenario':48s} | {'With smart exit':24s} | {'Without smart exit':24s}")
    print("-" * 102)
    for sc_key, sc_meta in SCENARIOS.items():
        r_with = [r for r in rows if r["scenario"] == sc_key and r["gate_mode"] == "with_smart_exit"]
        r_without = [r for r in rows if r["scenario"] == sc_key and r["gate_mode"] == "without_smart_exit"]

        def fmt_cell(rl):
            tet_m = st.mean(r["TET"] for r in rl)
            aet_m = st.mean(r["AET"] for r in rl)
            tpid_m = st.mean(r["TPID"] for r in rl)
            return f"TET: {tet_m:5.2f}s, AET: {aet_m:5.2f}s, TPID: {tpid_m:4.1f}"

        print(f"{sc_meta['title']:48s} | {fmt_cell(r_with)} | {fmt_cell(r_without)}")
    print("=" * 102)
    print(f"Saved {len(rows)} CSV + {len(rows)} JSON files in: {runs_dir} (elapsed {time.time() - t0:.1f}s)")

    if not args.skip_videos:
        video_tasks = [
            (sc_key, gate_key, 0, video_dir, args.fps, args.dpi)
            for sc_key in SCENARIOS
            for gate_key in GATE_MODES
        ]
        print(f"\nRendering {len(video_tasks)} MP4 videos (seed 0 for each scenario x gate state)...")
        t1 = time.time()
        with ProcessPoolExecutor(max_workers=min(args.workers, len(video_tasks))) as ex:
            for i, mp4_path in enumerate(ex.map(render_single_video, video_tasks), 1):
                print(f"  [{i:2d}/{len(video_tasks)}] saved {mp4_path}", flush=True)
        print(f"All {len(video_tasks)} videos written to: {video_dir} (elapsed {time.time() - t1:.1f}s)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
