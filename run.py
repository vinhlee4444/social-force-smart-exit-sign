"""
Run one evacuation.

Examples
--------
  python run.py                                  # live window, 200 agents, gates on
  python run.py --save results/run.mp4           # write a video (also .gif)
  python run.py --headless --out results/run1    # no graphics, just CSV/JSON/plots
  python run.py --no-gates --save results/baseline.mp4
  python run.py --agents 300 --seed 7 --fps 20   # 2x speed video

Every run writes timeseries.csv, summary.json and timeseries.png to --out.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def build_parser() -> argparse.ArgumentParser:
    from sfm.config import Config
    d = Config()
    p = argparse.ArgumentParser(
        description="SFM evacuation with smart exit gates (defaults come from sfm/config.py)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--agents", type=int, default=d.spawn.n_agents)
    p.add_argument("--seed", type=int, default=d.sim.seed)
    p.add_argument("--no-gates", action="store_true", help="baseline: gates never close")
    p.add_argument("--close-above", type=float, default=d.gate.close_above,
                   help="close threshold, in the unit of --metric")
    p.add_argument("--reopen-at-most", type=float, default=d.gate.reopen_at_most,
                   help="reopen threshold, in the unit of --metric")
    p.add_argument("--metric", choices=["density", "count"], default=d.sensor.metric,
                   help="what the gate reads: people/m2 of visible floor, or raw people count")
    p.add_argument("--smooth", type=float, default=d.sensor.smooth_window,
                   help="moving-average window of the camera reading, seconds (0 = off)")
    p.add_argument("--fov", type=float, default=d.sensor.fov_deg,
                   help="camera horizontal FOV in degrees (360 = old circular zone)")
    p.add_argument("--mount-height", type=float, default=d.sensor.mount_height, help="metres")
    p.add_argument("--tilt", type=float, default=d.sensor.tilt_deg, help="degrees below horizontal")
    p.add_argument("--no-min-range", action="store_true", help="ignore the blind zone under the camera")
    p.add_argument("--min-closed", type=float, default=d.gate.min_closed_time, help="seconds")
    p.add_argument("--max-closed", type=float, default=d.gate.max_closed_time,
                   help="fail-safe: reopen after this many seconds whatever the crowd")
    p.add_argument("--min-open", type=float, default=d.gate.min_open_time,
                   help="seconds a gate stays open after reopening (anti-flicker)")
    p.add_argument("--close-warning", type=float, default=d.gate.close_warning_time,
                   help="seconds of CLOSING warning before the gate shuts (0 = instant)")
    p.add_argument("--mode", choices=["block", "divert"], default=d.gate.mode,
                   help="block = shut the door; divert = keep it open, sign redirects people not yet in the queue")
    p.add_argument("--commit-distance", type=float, default=d.gate.commit_distance,
                   help="metres: in divert mode nobody closer than this to the door turns back")
    p.add_argument("--cost-divert", action="store_true",
                   help="divert mode: compare walk + queue time before switching (default: go to the nearest open exit)")
    p.add_argument("--flow", type=float, default=d.gate.flow_per_m_s,
                   help="assumed door flow, people / s / m of clear width (queue-wait estimate)")
    p.add_argument("--switch-margin", type=float, default=d.gate.switch_margin_s,
                   help="seconds an alternative exit must be faster by before people switch")
    p.add_argument("--min-open-exits", type=int, default=d.gate.min_open_exits,
                   help="never close a gate if fewer than this many OTHER exits are passable")
    p.add_argument("--notice-distance", type=float, default=d.gate.notice_distance,
                   help="metres: agents only notice a closed gate within this walking distance")
    p.add_argument("--period", type=float, default=d.sensor.period, help="seconds between camera readings")
    p.add_argument("--count-error", type=float, default=d.sensor.count_error,
                   help="relative std of the AI people count (0 = perfect)")
    p.add_argument("--radius", type=float, default=d.sensor.radius, help="camera range in metres (8-10)")
    p.add_argument("--no-los", action="store_true", help="count through walls (plain circle)")
    p.add_argument("--no-reroute", action="store_true",
                   help="agents keep their original exit even if its gate closes")
    p.add_argument("--t-max", type=float, default=d.sim.t_max)
    p.add_argument("--dt", type=float, default=d.sim.dt)
    p.add_argument("--map", default=d.map.image_path)
    p.add_argument("--door-width", type=float, default=None,
                   help="narrow every exit door to this clear width in metres (default: as drawn, 5.6 m)")
    p.add_argument("--spawn-x-min", type=float, default=None, help="spawn agents only at x >= this (m)")
    p.add_argument("--spawn-x-max", type=float, default=None, help="spawn agents only at x <= this (m)")
    p.add_argument("--headless", action="store_true", help="no window, no video")
    p.add_argument("--save", default=None, help="output video path (.mp4 or .gif)")
    p.add_argument("--fps", type=int, default=10, help="video fps (10 = real time)")
    p.add_argument("--dpi", type=int, default=80)
    p.add_argument("--out", default=None, help="results folder (default: results/<timestamp>)")
    return p


def make_config(a):
    from sfm.config import Config
    cfg = Config()
    cfg.map.image_path = a.map
    cfg.map.exit_door_width_m = a.door_width
    cfg.spawn.x_min_m = a.spawn_x_min
    cfg.spawn.x_max_m = a.spawn_x_max
    cfg.spawn.n_agents = a.agents
    cfg.sim.seed = a.seed
    cfg.sim.dt = a.dt
    cfg.sim.t_max = a.t_max
    if a.no_gates:
        cfg.gate.enabled = False
    cfg.gate.close_above = a.close_above
    cfg.gate.reopen_at_most = a.reopen_at_most
    cfg.gate.min_closed_time = a.min_closed
    cfg.gate.max_closed_time = a.max_closed
    cfg.gate.min_open_time = a.min_open
    cfg.gate.close_warning_time = a.close_warning
    cfg.gate.min_open_exits = a.min_open_exits
    cfg.gate.mode = a.mode
    cfg.gate.cost_based_divert = a.cost_divert
    cfg.gate.flow_per_m_s = a.flow
    cfg.gate.switch_margin_s = a.switch_margin
    cfg.gate.commit_distance = a.commit_distance
    cfg.gate.notice_distance = a.notice_distance
    cfg.sensor.period = a.period
    cfg.sensor.count_error = a.count_error
    if a.no_reroute:
        cfg.gate.reroute_on_close = False
    cfg.sensor.radius = a.radius
    cfg.sensor.metric = a.metric
    cfg.sensor.smooth_window = a.smooth
    cfg.sensor.fov_deg = a.fov
    cfg.sensor.mount_height = a.mount_height
    cfg.sensor.tilt_deg = a.tilt
    if a.no_min_range:
        cfg.sensor.use_min_range = False
    if a.no_los:
        cfg.sensor.line_of_sight = False
    return cfg


def main(argv=None) -> int:
    a = build_parser().parse_args(argv)
    if a.headless or a.save:
        import matplotlib
        matplotlib.use("Agg")

    from sfm.simulation import Simulation

    cfg = make_config(a)
    out = a.out or os.path.join(BASE_DIR, "results", time.strftime("%Y%m%d-%H%M%S"))
    sim = Simulation(cfg, base_dir=BASE_DIR)
    print(f"{sim.n_total} agents spawned in {int(sim.rooms.max()) if sim.rooms is not None else '?'} rooms; "
          f"gates {'ON' if cfg.gate.enabled else 'OFF'}; seed {cfg.sim.seed}")

    t0 = time.time()
    if a.save:
        from sfm.render import Renderer
        os.makedirs(os.path.dirname(os.path.abspath(a.save)), exist_ok=True)
        Renderer(sim).save(a.save, fps=a.fps, dpi=a.dpi)
        print(f"video written to {a.save}")
    elif a.headless:
        sim.run()
    else:
        from sfm.render import Renderer
        Renderer(sim).show()
        if not sim.done:
            print("window closed before the evacuation finished; saving partial results")

    sim.save_results(out)
    s = sim.recorder.summary()
    print(f"\nwall-clock {time.time() - t0:.1f} s | sim time {s['sim_time_s']} s | "
          f"evacuated {s['evacuated']}/{s['n_agents']} | T90 {s['T90_s']} s | T100 {s['T100_s']} s")
    for e in s["exits"]:
        print(f"  {e['name']:16s} evacuated {e['evacuated']:4d} | closures {e['closures']:2d} | "
              f"closed {e['total_closed_s']:6.1f} s | max count {e['max_count']} | "
              f"max value {e['max_value']:g} | area {e['visible_area_m2']} m2")
    print(f"  peak agents in danger {s['peak_agents_in_danger']} | danger exposure {s['danger_person_seconds']} person-s")
    print(f"results in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

