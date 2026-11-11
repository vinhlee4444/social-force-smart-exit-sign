"""
Recorder: time series + summary statistics of one run.
"""
from __future__ import annotations

import csv
import json
from typing import TYPE_CHECKING, Dict, List

import numpy as np

if TYPE_CHECKING:
    from .simulation import Simulation


class Recorder:
    def __init__(self, sim: "Simulation"):
        self.sim = sim
        self.rows: List[Dict] = []
        self.peak_contact = 0.0

    def record(self) -> None:
        sim = self.sim
        c = sim.crowd
        max_contact = float(c.contact.max()) if len(c) else 0.0
        self.peak_contact = max(self.peak_contact, max_contact)
        n_danger = int(np.sum(c.contact > sim.cfg.sim.danger_force_N)) if len(c) else 0
        row = {"t": round(sim.t, 4), "remaining": len(c),
               "evacuated": int(np.sum(sim.evac_exit >= 0)), "max_contact_N": round(max_contact, 1),
               "n_danger": n_danger}
        for s in sim.signs:
            row[f"count_{s.idx + 1}"] = s.last_count
            row[f"value_{s.idx + 1}"] = round(s.last_value, 4)
            row[f"closed_{s.idx + 1}"] = int(s.closed)
            row[f"evac_{s.idx + 1}"] = int(np.sum(sim.evac_exit == s.idx))
        self.rows.append(row)

    # ------------------------------------------------------------------ output
    def summary(self) -> Dict:
        sim = self.sim
        times = np.sort(sim.evac_time[np.isfinite(sim.evac_time)])
        n = sim.n_total

        def t_frac(f):
            k = int(np.ceil(f * n))
            return float(times[k - 1]) if 0 < k <= len(times) else None

        t_end = sim.t
        signs = []
        for s in sim.signs:
            ev = s.gate.events
            closed_time, t_close = 0.0, None
            for e in ev:
                if e.closed:
                    t_close = e.t
                elif t_close is not None:
                    closed_time += e.t - t_close
                    t_close = None
            if t_close is not None:
                closed_time += t_end - t_close
            counts = [r[f"count_{s.idx + 1}"] for r in self.rows]
            values = [r[f"value_{s.idx + 1}"] for r in self.rows]
            signs.append({
                "name": s.name,
                "evacuated": int(np.sum(sim.evac_exit == s.idx)),
                "closures": sum(1 for e in ev if e.closed),
                "total_closed_s": round(closed_time, 2),
                "max_count": int(max(counts)) if counts else 0,
                "max_value": round(float(max(values)), 4) if values else 0.0,
                "visible_area_m2": round(s.visible_area_m2, 1),
                "events": [{"t": round(e.t, 2), "state": "CLOSED" if e.closed else "OPEN",
                            "value": round(e.count, 4), "reason": e.reason} for e in ev],
            })
        return {
            "n_agents": n,
            "evacuated": int(len(times)),
            "sim_time_s": round(t_end, 2),
            "all_evacuated": bool(len(times) == n),
            "T50_s": t_frac(0.5), "T90_s": t_frac(0.9), "T100_s": t_frac(1.0),
            "peak_contact_force_N": round(self.peak_contact, 1),
            # people squeezed above danger_force_N: peak head-count, and exposure in person-seconds
            "peak_agents_in_danger": int(max((r["n_danger"] for r in self.rows), default=0)),
            "danger_person_seconds": round(sum(r["n_danger"] for r in self.rows) * sim.cfg.sim.frame_period, 1),
            "gates_enabled": sim.cfg.gate.enabled,
            "seed": sim.cfg.sim.seed,
            "exits": signs,
        }

    def to_csv(self, path: str) -> None:
        if not self.rows:
            return
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)

    def save_summary(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.summary(), f, indent=2)

    def plot(self, path: str) -> None:
        import matplotlib.pyplot as plt

        sim = self.sim
        t = np.array([r["t"] for r in self.rows])
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axes[0].plot(t, [r["remaining"] for r in self.rows], "k-", label="remaining in building")
        for s in sim.signs:
            axes[0].plot(t, [r[f"evac_{s.idx + 1}"] for r in self.rows], label=f"evacuated via {s.name}")
        axes[0].set_ylabel("people")
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)

        for s in sim.signs:
            axes[1].plot(t, [r[f"value_{s.idx + 1}"] for r in self.rows], label=s.name)
        g = sim.cfg.gate
        axes[1].axhline(g.close_above, color="r", ls="--", lw=1, label=f"close > {g.close_above:g}")
        axes[1].axhline(g.reopen_at_most, color="g", ls="--", lw=1, label=f"reopen <= {g.reopen_at_most:g}")
        axes[1].set_ylabel("camera density (people/m2)" if sim.cfg.sensor.metric == "density"
                           else "camera count")
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)

        for s in sim.signs:
            closed = np.array([r[f"closed_{s.idx + 1}"] for r in self.rows])
            axes[2].fill_between(t, s.idx, s.idx + 0.8 * closed, step="post", alpha=0.7)
        axes[2].set_yticks(np.arange(len(sim.signs)) + 0.4)
        axes[2].set_yticklabels([s.name for s in sim.signs])
        axes[2].set_title("gate CLOSED periods", fontsize=9)
        axes[2].set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
