"""
Rendering: live matplotlib window, or a .gif / .mp4 file.

One frame = SimConfig.frame_period of simulated time (0.1 s -> 10 frames per sim second).
Playback fps = 10 gives real-time video; fps = 20 plays at 2x speed, etc.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import EllipseCollection
from matplotlib.patches import Rectangle

if TYPE_CHECKING:
    from .simulation import Simulation

EXIT_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:brown", "tab:pink"]


class Renderer:
    def __init__(self, sim: "Simulation", show_sensors: bool = True, show_plot: bool = True):
        self.sim = sim
        fl = sim.floor
        W, H = fl.width_m, fl.height_m

        if show_plot:
            self.fig, (self.ax, self.ax_ts) = plt.subplots(
                2, 1, figsize=(15, 8.6), gridspec_kw={"height_ratios": [3.2, 1]})
        else:
            self.fig, self.ax = plt.subplots(figsize=(15, 6.6))
            self.ax_ts = None
        ax = self.ax
        ax.imshow(np.where(fl.base_free, 1.0, 0.15), cmap="gray", vmin=0, vmax=1,
                  extent=[0, W, H, 0], interpolation="nearest")

        # Sensor coverage (what each camera can see).
        if show_sensors:
            overlay = np.zeros((fl.H, fl.W, 4))
            for s in sim.signs:
                rgba = plt.matplotlib.colors.to_rgba(EXIT_COLORS[s.idx % len(EXIT_COLORS)])
                overlay[s.mask] = (*rgba[:3], 0.18)
            ax.imshow(overlay, extent=[0, W, H, 0], interpolation="nearest")
            for s in sim.signs:
                ax.plot(*s.camera, marker="^", color=EXIT_COLORS[s.idx % len(EXIT_COLORS)],
                        markersize=8, markeredgecolor="k")

        # Gates and labels.
        self.gate_patches, self.labels = [], []
        for s in sim.signs:
            x0, y0, x1, y1 = s.door_m
            p = Rectangle((x0, y0), x1 - x0, y1 - y0, color="limegreen", zorder=5)
            ax.add_patch(p)
            self.gate_patches.append(p)
            ly = y0 - 2.5 if s.inward[1] > 0 else y1 + 2.5
            lbl = ax.text(s.door_center[0] + 4, ly, "", fontsize=9, zorder=6,
                          va="center", ha="left",
                          bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="0.5", alpha=0.85))
            self.labels.append(lbl)

        # Agents drawn at their true physical size.
        self.agents = EllipseCollection(
            widths=[], heights=[], angles=[], units="xy", offsets=np.zeros((0, 2)),
            offset_transform=ax.transData, edgecolors="k", linewidths=0.3, zorder=4)
        ax.add_collection(self.agents)
        self.title = ax.set_title("")
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        if self.ax_ts is not None:
            a = self.ax_ts
            self.lines = [a.plot([], [], color=EXIT_COLORS[s.idx % len(EXIT_COLORS)], label=s.name)[0]
                          for s in sim.signs]
            g = sim.cfg.gate
            a.axhline(g.close_above, color="r", ls="--", lw=1)
            a.axhline(g.reopen_at_most, color="g", ls="--", lw=1)
            a.set_ylabel("camera density (people/m2)" if sim.cfg.sensor.metric == "density"
                         else "camera count")
            a.set_xlabel("time (s)")
            a.legend(loc="upper right", fontsize=8, ncol=len(sim.signs))
            a.grid(alpha=0.3)
            self.ts_t, self.ts_c = [], [[] for _ in sim.signs]
        self.fig.tight_layout()
        self.update()

    def update(self):
        sim, c = self.sim, self.sim.crowd
        d = 2 * c.radius
        self.agents.set_offsets(c.pos if len(c) else np.zeros((0, 2)))
        self.agents.set_widths(d)
        self.agents.set_heights(d)
        self.agents.set_angles(np.zeros(len(c)))
        self.agents.set_facecolors([EXIT_COLORS[k % len(EXIT_COLORS)] for k in c.target])

        g = sim.cfg.gate
        for s, p, lbl in zip(sim.signs, self.gate_patches, self.labels):
            p.set_color("red" if s.closed else "orange" if s.state == "CLOSING" else "limegreen")
            state = "REDIRECT" if (s.closed and sim.cfg.gate.mode == "divert") else s.state
            val = f" ({s.last_value:.2f}/m2)" if sim.cfg.sensor.metric == "density" else ""
            lbl.set_text(f"E{s.idx + 1}: {s.last_count:2d} seen{val} | {state}")
            lbl.get_bbox_patch().set_edgecolor("red" if s.closed else "0.5")
        evac = int(np.sum(sim.evac_exit >= 0))
        self.title.set_text(f"Social Force Model evacuation  |  t = {sim.t:6.1f} s  |  "
                            f"inside: {len(c)}  evacuated: {evac}/{sim.n_total}  |  "
                            f"gate closes > {g.close_above:g}, reopens <= {g.reopen_at_most:g} "
                            f"after {g.min_closed_time:.0f} s" + ("" if g.enabled else "  [GATES DISABLED]"))

        if self.ax_ts is not None:
            self.ts_t.append(sim.t)
            for k, s in enumerate(sim.signs):
                self.ts_c[k].append(s.last_value)
                self.lines[k].set_data(self.ts_t, self.ts_c[k])
            self.ax_ts.set_xlim(0, max(10.0, sim.t * 1.05))
            self.ax_ts.set_ylim(0, max(g.close_above * 1.3, max(max(cs) for cs in self.ts_c) * 1.1))
        return [self.agents, self.title, *self.gate_patches, *self.labels]

    # ------------------------------------------------------------------ outputs
    def save(self, path: str, fps: int = 10, dpi: int = 80, extra_end_frames: int = 10,
             progress: bool = True) -> None:
        """Run the simulation to the end while writing every frame to a .gif or .mp4."""
        from matplotlib import animation
        if path.lower().endswith(".gif"):
            writer = animation.PillowWriter(fps=fps)
        else:
            try:
                import imageio_ffmpeg
                plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError:
                pass
            writer = animation.FFMpegWriter(fps=fps, codec="libx264",
                                            extra_args=["-pix_fmt", "yuv420p", "-crf", "23"])
        sim = self.sim
        last_print = -10.0
        with writer.saving(self.fig, path, dpi):
            writer.grab_frame()
            while not sim.done:
                sim.advance_frame()
                self.update()
                writer.grab_frame()
                if progress and sim.t - last_print >= 10.0:
                    last_print = sim.t
                    print(f"  t={sim.t:6.1f}s  remaining={len(sim.crowd):4d}  "
                          f"counts={[s.last_count for s in sim.signs]}  "
                          f"closed={[int(s.closed) for s in sim.signs]}", flush=True)
            for _ in range(extra_end_frames):
                writer.grab_frame()
        plt.close(self.fig)

    def show(self, interval_ms: int = 30) -> None:
        """Live window. Each animation frame advances one simulation frame."""
        from matplotlib.animation import FuncAnimation

        def frames():
            while not self.sim.done:
                yield None

        def step(_):
            self.sim.advance_frame()
            return self.update()

        self._anim = FuncAnimation(self.fig, step, frames=frames, interval=interval_ms,
                                   blit=False, repeat=False, cache_frame_data=False)
        plt.show()

