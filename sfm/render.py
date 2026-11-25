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
        ax.imshow(fl.source_raster, cmap="gray", vmin=0, vmax=255,
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
            p.set_color({"RED": "red", "GREEN": "limegreen"}.get(s.state, "gray"))
            state = s.state
            val = f" ({s.last_value:.2f}/m2)" if sim.cfg.sensor.metric == "density" else ""
            lbl.set_text(f"E{s.idx + 1}: {s.last_count:2d} seen{val} | {state}")
            lbl.get_bbox_patch().set_edgecolor("red" if s.closed else "0.5")
        evac = int(np.sum(sim.evac_exit >= 0))
        self.title.set_text(f"Social Force Model evacuation  |  t = {sim.t:6.1f} s  |  "
                            f"inside: {len(c)}  destination reached: {evac}/{sim.n_total}  |  "
                            f"RED > {g.close_above:g}, GREEN <= {g.reopen_at_most:g} "
                            f"after {g.min_closed_time:.0f} s" + ("" if g.enabled else "  [GUIDANCE OFF]"))

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



class ComparisonPanel:
    """Raster panel for one side of a synchronized study video."""

    width, height = 768, 560
    left, top, map_width = 24, 92, 720
    people_colors = [(215, 128, 45), (30, 124, 247), (190, 102, 170), (200, 179, 26)]
    signals = {"OFF": (112, 117, 128), "RED": (48, 65, 220), "GREEN": (57, 163, 38)}

    def __init__(self, sim, label, quota):
        import cv2

        self.sim, self.label, self.quota = sim, label, quota
        floor = sim.floor
        self.scale = min(self.map_width / floor.width_m,
                         (self.height - self.top - 80) / floor.height_m)
        self.map_width = round(floor.width_m * self.scale)
        self.left = (self.width - self.map_width) // 2
        height = round(floor.height_m * self.scale)
        self.map_end = self.top + height
        base = cv2.cvtColor(floor.source_raster, cv2.COLOR_GRAY2BGR)
        for sign, color in zip(sim.signs, self.people_colors):
            base[sign.mask] = (0.65 * base[sign.mask] + 0.35 * np.asarray(color)).astype(np.uint8)
        self.base = np.full((self.height, self.width, 3), (30, 25, 22), dtype=np.uint8)
        self.base[self.top:self.map_end, self.left:self.left + self.map_width] = cv2.resize(
            base, (self.map_width, height), interpolation=cv2.INTER_NEAREST)
        # Each thin white dash marks the spawn quota boundary.
        x, _ = self.pixel((floor.split_x_m, 0))
        for y in range(self.top, self.map_end, 10):
            cv2.line(self.base, (x, y), (x, min(y + 3, self.map_end - 1)), (165, 152, 137), 1)
        self.final = None
        self.recent_switches = {}
        self.read_events = 0

    def pixel(self, pos):
        return tuple((np.asarray(pos) * self.scale + [self.left, self.top]).round().astype(int))

    @staticmethod
    def write(frame, text, x, y, scale=0.57, color=(239, 232, 224), thickness=1):
        import cv2
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

    def frame(self):
        import cv2

        if self.final is not None:
            return self.final
        sim, c = self.sim, self.sim.crowd
        frame = self.base.copy()
        title = "OFF / Nearest exit" if self.label == "OFF" else "ON / Congestion guidance"
        self.write(frame, title, 24, 32, 0.84, thickness=2)
        remaining = len(c)
        status = ("COMPLETED" if not remaining else "TIMEOUT") if sim.done else "RUNNING"
        left_share = round(100 * self.quota[0] / sim.n_total)
        self.write(frame, f"N {sim.n_total}   L/R {left_share}/{100 - left_share}%   "
                   f"t {sim.t:6.2f} s   Remaining {remaining}   Reached {sim.n_total - remaining}   {status}",
                   24, 62)
        self.write(frame, f"Map {sim.floor.image_name}   |   SHA-256 {sim.floor.image_sha256[:16]}",
                   24, 83, 0.46)
        guidance = sim.guidance
        message = "Camera densities shown for comparison   |   Guidance disabled"
        if guidance is not None:
            for t, agent, previous, current in guidance.events[self.read_events:]:
                self.recent_switches[agent] = (t, previous, current)
            self.read_events = len(guidance.events)
            self.recent_switches = {
                agent: event for agent, event in self.recent_switches.items()
                if sim.t - event[0] <= 3
            }
            gate = sim.cfg.gate
            message = (f"RED > {gate.close_above:g} / GREEN <= {gate.reopen_at_most:g} people/m^2"
                       "   |   Outlined = recently switched")
        labels = []
        for agent, pos, radius, target in zip(c.ids, c.pos, c.radius, c.target):
            p = self.pixel(pos)
            r = max(1, round(radius * self.scale))
            color = self.people_colors[target]
            cv2.circle(frame, p, r, color, -1, cv2.LINE_AA)
            cv2.circle(frame, p, r, (41, 35, 31), 1, cv2.LINE_AA)
            event = self.recent_switches.get(agent)
            if event is not None:
                _, previous, current = event
                cv2.circle(frame, p, r + 3, (255, 255, 255), 2, cv2.LINE_AA)
                labels.append((p, r, f"ID {agent}  E{previous + 1} -> E{current + 1}"))
        for (px, py), r, label in labels:
            (width, height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            x = max(self.left + 2, min(px - width // 2, self.left + self.map_width - width - 4))
            y = max(self.top + height + 5, py - r - 7)
            cv2.rectangle(frame, (x - 2, y - height - 2), (x + width + 2, y + 3), (30, 25, 22), -1)
            self.write(frame, label, x, y, 0.4)
        for sign in sim.signs:
            color = self.signals[sign.state]
            x0, y0, x1, y1 = sign.door_m
            a, b = self.pixel((x0, y0)), self.pixel((x1, y1))
            cv2.rectangle(frame, a, b, color, 2, cv2.LINE_AA)
            p = self.pixel(sign.camera)
            cv2.circle(frame, p, 4, color, -1, cv2.LINE_AA)
            label = f"E{sign.idx + 1}"
            if guidance is not None:
                destinations, _ = guidance.recommendations(sign.idx, sim.t)
                if destinations:
                    label += " -> " + "/".join(f"E{k + 1}" for k in destinations)
            (width, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.43, 1)
            label_x = p[0] + 12
            if label_x + width >= self.left + self.map_width - 3:
                label_x = max(self.left + 3, p[0] - width - 11)
            self.write(frame, label, label_x, p[1], 0.43, (30, 25, 22), 1)
            x = 24 + (sign.idx % 2) * 360
            y = self.map_end + 22 + (sign.idx // 2) * 24
            cv2.circle(frame, (x + 4, y - 5), 5, color, -1, cv2.LINE_AA)
            cv2.circle(frame, (x + 22, y - 5), 4, self.people_colors[sign.idx], -1, cv2.LINE_AA)
            self.write(frame, f"E{sign.idx + 1}   {sign.last_value:.2f} people/m^2   {sign.state}",
                       x + 34, y, 0.52)
        self.write(frame, message, 24, self.map_end + 72, 0.48)
        if sim.done:
            self.final = frame
        return frame


def _comparison_summary(size, off, on, quota, floor, on_policy=None):
    from matplotlib import get_data_path
    from pathlib import Path
    from PIL import Image, ImageDraw, ImageFont

    image = Image.new("RGB", size, (22, 25, 30))
    draw = ImageDraw.Draw(image)
    font_path = Path(get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
    font = ImageFont.truetype(str(font_path), 24)
    heading = ImageFont.truetype(str(font_path), 35)
    fit = lambda value: "ch\u01b0a \u0111\u1ea1t" if value is None else f"{value:.2f} s"
    draw.text((100, 40), f"Destination-zone results  /  N {sum(quota)}  /  L/R {quota[0]}/{quota[1]}",
              font=heading, fill="#f3eee7")
    draw.text((100, 97), "Arrival is 1 m beyond the door in the floor-level destination zone.",
              font=font, fill="#bdb5a8")
    rows = [
        ("", "OFF", f"ON / {on_policy}" if on_policy else "ON"),
        ("Status", off["status"], on["status"]),
        ("T90", fit(off["T90_s"]), fit(on["T90_s"])),
        ("T100", fit(off["T100_s"]), fit(on["T100_s"])),
        ("Remaining", off["remaining"], on["remaining"]),
        ("Restricted mean (at study limit)", fit(off["restricted_mean_s"]), fit(on["restricted_mean_s"])),
        ("Target switches", off["switches"], on["switches"]),
    ]
    for i, (label, a, b) in enumerate(rows):
        y = 154 + i * 46
        if i % 2:
            draw.rectangle((90, y - 4, size[0] - 90, y + 38), fill="#2a303a")
        for x, text, ink in zip((108, 800, 1120), (label, a, b), ("#f3eee7", "#afd5f6", "#b2e6b0")):
            draw.text((x, y), str(text), font=font, fill=ink)
    if off["T100_s"] and on["T100_s"] is not None:
        delta = (off["T100_s"] - on["T100_s"]) / off["T100_s"] * 100
        draw.text((108, 493), f"T100 reduction (ON vs OFF): {delta:+.2f}%", font=font, fill="#f3eee7")
    map_font = ImageFont.truetype(str(font_path), 16)
    draw.text((108, 529), f"Map {floor.image_name}   |   SHA-256 {floor.image_sha256}",
              font=map_font, fill="#bdb5a8")
    return np.asarray(image)


def _comparison_panels(off, on, quota):
    if (off.floor.image_sha256 != on.floor.image_sha256 or off.floor.s != on.floor.s
            or not np.array_equal(off.floor.base_free, on.floor.base_free)):
        raise ValueError("OFF and ON must render the same floor plan")
    return [ComparisonPanel(off, "OFF", quota), ComparisonPanel(on, "ON", quota)]


def _paired_frame(panels):
    return np.concatenate([panel.frame() for panel in panels], axis=1)


def comparison_preview(off, on, quota):
    """Return the same initial BGR frame used by the MP4 writer."""
    return _paired_frame(_comparison_panels(off, on, quota))


def save_comparison(off, on, output, quota, fps=10, sample_s=0.5, first_frame_path=None):
    """Stream the paired MP4, decode it, then atomically replace the requested output."""
    import math
    import os
    import subprocess
    from tempfile import NamedTemporaryFile

    import imageio_ffmpeg

    panels = _comparison_panels(off, on, quota)
    expected_first = _paired_frame(panels)
    size = (2 * panels[0].width, panels[0].height)
    frames_per_sample = round(sample_s / off.cfg.sim.frame_period)
    output.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(dir=output.parent, suffix=".mp4", delete=False) as file:
        temp = file.name
    staged_png = None
    try:
        writer = imageio_ffmpeg.write_frames(
            temp, size, fps=fps, codec="libx264", pix_fmt_in="rgb24", macro_block_size=1,
            output_params=["-preset", "veryfast", "-movflags", "+faststart"])
        writer.send(None)
        count = 0
        try:
            last_print = -30
            while True:
                pair = expected_first if count == 0 else _paired_frame(panels)
                writer.send(np.ascontiguousarray(pair[:, :, ::-1]))
                count += 1
                t = max(off.t, on.t)
                if off.done and on.done:
                    break
                if t - last_print >= 30:
                    print(f"  t={t:.1f}s, remaining OFF/ON={len(off.crowd)}/{len(on.crowd)}", flush=True)
                    last_print = t
                for panel in panels:
                    for _ in range(frames_per_sample):
                        panel.sim.advance_frame()
            summaries = off.recorder.summary(), on.recorder.summary()
            end_card = _comparison_summary(size, *summaries, quota, off.floor, on.cfg.guidance.policy)
            card_frames = 5 * fps
            for _ in range(card_frames):
                writer.send(end_card)
                count += 1
        finally:
            writer.close()
        check = subprocess.run(
            [imageio_ffmpeg.get_ffmpeg_exe(), "-v", "error", "-xerror", "-i", temp,
             "-progress", "pipe:1", "-nostats", "-f", "null", "-"],
            capture_output=True, check=True, text=True)
        decoded = [int(line.split("=")[1]) for line in check.stdout.splitlines() if line.startswith("frame=")]
        steps_per_sample = frames_per_sample * off.frame_every
        expected = 1 + math.ceil(max(off.step_idx, on.step_idx) / steps_per_sample) + card_frames
        if count != expected or not decoded or decoded[-1] != expected:
            raise RuntimeError(f"Expected {expected} frames of paired video, wrote {count}, "
                               f"decoded {decoded[-1] if decoded else 0}")
        _verify_end_card(temp, end_card, expected, card_frames, fps)
        actual_first = _verify_first_frame(temp, expected_first)
        if first_frame_path is not None:
            import cv2

            ok, png = cv2.imencode(".png", actual_first)
            if not ok:
                raise RuntimeError("Could not encode the decoded first frame")
            with NamedTemporaryFile(dir=output.parent, suffix=".png", delete=False) as file:
                staged_png = file.name
                file.write(png.tobytes())
        os.replace(temp, output)
        if staged_png is not None:
            os.replace(staged_png, first_frame_path)
        return summaries
    finally:
        if os.path.exists(temp):
            os.unlink(temp)
        if staged_png is not None and os.path.exists(staged_png):
            os.unlink(staged_png)


def _verify_first_frame(path, reference):
    """Check the decoded layout and initial scene against the renderer's preview."""
    import cv2

    cap = cv2.VideoCapture(path)
    try:
        ok, frame = cap.read()
        if not ok or frame.shape != reference.shape:
            raise RuntimeError("Could not decode the expected first frame")
        difference = np.abs(frame.astype(np.int16) - reference)
        if difference.mean() > 5 or (difference.max(axis=2) > 30).mean() > 0.03:
            raise RuntimeError("Decoded first frame does not match the rendered map preview")
        return frame
    finally:
        cap.release()


def _verify_end_card(path, expected_card, total_frames, card_frames, fps):
    """Verify the encoded summary at the start and end of its display interval."""
    import cv2

    cap = cv2.VideoCapture(path)
    reference = expected_card[:, :, ::-1].astype(np.int16)
    text_pixels = reference.max(axis=2) > 140
    try:
        if (not cap.isOpened() or cap.get(cv2.CAP_PROP_FRAME_COUNT) != total_frames
                or not np.isclose(cap.get(cv2.CAP_PROP_FPS), fps)):
            raise RuntimeError("Paired MP4 metadata did not match the expected frames and FPS")
        for index in (total_frames - card_frames, total_frames - 1):
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            if not ok or frame.shape != reference.shape:
                raise RuntimeError(f"Could not read summary frame {index}")
            difference = np.abs(frame - reference)
            if difference.mean() > 5 or difference[text_pixels].mean() > 24:
                raise RuntimeError(f"Frame {index} does not match the results table")
    finally:
        cap.release()
