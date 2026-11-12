"""
Overload scenarios: does the smart gate help when an exit is REALLY overloaded?

  python scenarios.py                         # everything: 4 scenarios x 5 variants x 4 seeds (parallel)
  python scenarios.py --seeds 2               # quicker
  python scenarios.py --only left_1m2         # one scenario  (uniform_1m2, uniform_1m0, left_1m2, left_1m0)
  python scenarios.py --variants "Chặn,Không" # only variants whose name contains one of these words

Every (scenario, variant, seed) is one simulation; they run in parallel. Prints a table and writes
results/scenarios.csv.
"""
from __future__ import annotations

import argparse
import copy
import csv
import os
import statistics as st
import sys
from concurrent.futures import ProcessPoolExecutor

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)

from sfm.config import Config                    # noqa: E402
from sfm.simulation import Simulation            # noqa: E402

# ---- scenarios: what the building looks like / who is where ---------------------------------
SCENARIOS = {
    "uniform_1m2":  dict(label="cửa 1.2m, 300 người phân bố đều",             door=1.2, n=300, x_max=None),
    "uniform_1m0":  dict(label="cửa 1.0m, 300 người phân bố đều",             door=1.0, n=300, x_max=None),
    "left_1m2":     dict(label="cửa 1.2m, 300 người dồn nửa trái (E2/E4 vắng)", door=1.2, n=300, x_max=60.0),
    "left_1m0":     dict(label="cửa 1.0m, 300 người dồn nửa trái (E2/E4 vắng)", door=1.0, n=300, x_max=60.0),
}


# ---- variants: what the gate does ------------------------------------------------------------
def v_nogate(c):
    c.gate.enabled = False


def v_old(c):                                    # original behaviour (circle, count 25/15, instant)
    c.sensor.fov_deg, c.sensor.use_min_range, c.sensor.radius = 360, False, 10
    c.sensor.metric, c.sensor.smooth_window, c.sensor.period, c.sensor.count_error = "count", 0, 0.1, 0
    c.gate.close_above, c.gate.reopen_at_most = 25, 15
    c.gate.min_open_time, c.gate.max_closed_time = 0, 1e9


def make_div(close, reopen):
    def f(c):
        c.gate.close_above, c.gate.reopen_at_most = close, reopen
    return f


VARIANTS = {
    "Không cổng":                         v_nogate,
    "Cổng cũ":                            v_old,                       # only with --with-old (it jams)
    "Chuyển hướng gần nhất 0.7/0.4":      make_div(0.7, 0.4),
    "Chuyển hướng gần nhất 0.5/0.3":      make_div(0.5, 0.3),
}


def run_one(args):
    sc_key, var_name, seed, t_max = args
    sc, c = SCENARIOS[sc_key], Config()
    c.map.exit_door_width_m, c.spawn.n_agents, c.spawn.x_max_m = sc["door"], sc["n"], sc["x_max"]
    c.sim.seed, c.sim.t_max = seed, t_max
    VARIANTS[var_name](c)
    sim = Simulation(c, BASE, verbose=False)
    s = sim.run(progress=False)
    return dict(scenario=sc_key, variant=var_name, seed=seed,
                done=s["all_evacuated"], evacuated=s["evacuated"], n=s["n_agents"],
                T90=s["T90_s"], T100=s["T100_s"], peak_N=s["peak_contact_force_N"],
                peak_danger=s["peak_agents_in_danger"], danger_ps=s["danger_person_seconds"],
                closures=sum(e["closures"] for e in s["exits"]))


def main():
    for stream in (sys.stdout, sys.stderr):          # Windows consoles/redirects may not be UTF-8
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--only", default=None, help="comma-separated scenario keys")
    ap.add_argument("--t-max", type=float, default=200.0)
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--variants", default=None, help="comma-separated substrings of variant names to keep")
    ap.add_argument("--with-old", action="store_true", help="also run the original gate logic (slow: it jams)")
    a = ap.parse_args()
    if not a.with_old:
        VARIANTS.pop("Cổng cũ", None)
    if a.variants:
        keep = a.variants.split(",")
        for name in list(VARIANTS):
            if not any(k in name for k in keep):
                VARIANTS.pop(name)
    keys = a.only.split(",") if a.only else list(SCENARIOS)

    # Build every route-field cache once, so the workers only load them.
    for k in keys:
        c = Config(); c.map.exit_door_width_m = SCENARIOS[k]["door"]; c.spawn.n_agents = 10
        c.spawn.x_max_m = SCENARIOS[k]["x_max"]
        Simulation(c, BASE, verbose=False)

    tasks = [(k, v, s, a.t_max) for k in keys for v in VARIANTS for s in range(a.seeds)]
    rows = []
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        for i, r in enumerate(ex.map(run_one, tasks), 1):
            rows.append(r)
            print(f"[{i}/{len(tasks)}] {r['scenario']} | {r['variant']} | seed {r['seed']} -> "
                  f"T100={r['T100']} danger={r['danger_ps']} closures={r['closures']}", file=sys.stderr, flush=True)

    os.makedirs(os.path.join(BASE, "results"), exist_ok=True)
    # utf-8-sig: Vietnamese names survive on Windows (default cp1252 cannot encode them) and Excel opens it
    with open(os.path.join(BASE, "results", "scenarios.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

    for k in keys:
        print(f"\n=== {SCENARIOS[k]['label']}  ({a.seeds} seed)")
        print(f"  {'cấu hình':20s} {'xong':>5s} {'T90':>7s} {'T100':>7s} {'nén đỉnh N':>11s} {'người nguy hiểm':>16s} {'nguy hiểm p.s':>14s} {'đóng':>5s}")
        for v in VARIANTS:
            R = [r for r in rows if r["scenario"] == k and r["variant"] == v]
            t90 = [r["T90"] for r in R if r["T90"]]; t100 = [r["T100"] for r in R if r["T100"]]
            fmt = lambda x: f"{st.mean(x):7.1f}" if x else "    n/a"
            print(f"  {v:20s} {sum(r['done'] for r in R):>2d}/{len(R)} {fmt(t90)} {fmt(t100)} "
                  f"{max(r['peak_N'] for r in R):11.0f} {st.mean(r['peak_danger'] for r in R):16.1f} "
                  f"{st.mean(r['danger_ps'] for r in R):14.0f} {st.mean(r['closures'] for r in R):5.1f}")


if __name__ == "__main__":
    main()
