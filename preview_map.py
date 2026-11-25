"""
Preview the floor plan with the smart exit sign cameras and their fields of view (FOV),
using the current parameters from `sfm/config.py`.

Usage:
  python preview_map.py                     # saves map_fov.png and opens a window
  python preview_map.py --save my_fov.png   # saves to custom path
  python preview_map.py --no-show           # only save the image, do not open a window
"""
from __future__ import annotations

import argparse
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from sfm.config import Config
from sfm.exit_sign import ExitSign
from sfm.floorplan import FloorPlan
from sfm.render import EXIT_COLORS

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    ap = argparse.ArgumentParser(description="Render floor plan and camera FOV cones from sfm/config.py")
    ap.add_argument("--save", default="map_fov.png", help="output PNG path (default: map_fov.png)")
    ap.add_argument("--no-show", action="store_true", help="do not pop up an interactive matplotlib window")
    ap.add_argument("--dpi", type=int, default=150, help="saved image DPI")
    args = ap.parse_args()

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")

    cfg = Config()
    fl = FloorPlan(cfg.map, cfg.exits, base_dir=BASE_DIR)
    signs = [
        ExitSign(i, ex, fl, cfg.sensor, cfg.gate, cfg.nav.evac_depth)
        for i, ex in enumerate(cfg.exits)
    ]

    W, H = fl.width_m, fl.height_m
    fig, ax = plt.subplots(figsize=(14, 7.5))
    ax.imshow(np.where(fl.base_free, 1.0, 0.15), cmap="gray", vmin=0, vmax=1,
              extent=[0, W, H, 0], interpolation="nearest")

    # Draw camera visible floor masks + crisp edges
    overlay = np.zeros((fl.H, fl.W, 4))
    ker = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    for s in signs:
        rgba = plt.matplotlib.colors.to_rgba(EXIT_COLORS[s.idx % len(EXIT_COLORS)])
        overlay[s.mask] = (*rgba[:3], 0.38)
        edge = s.mask ^ cv2.erode(s.mask.astype(np.uint8), ker).astype(bool)
        overlay[edge] = (*rgba[:3], 0.95)
    ax.imshow(overlay, extent=[0, W, H, 0], interpolation="nearest", zorder=2)

    # Draw camera positions, doors, and info labels
    for s in signs:
        color = EXIT_COLORS[s.idx % len(EXIT_COLORS)]
        x0, y0, x1, y1 = s.door_m
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="limegreen", zorder=5))
        ax.plot(*s.camera, marker="^", color=color, markersize=9, markeredgecolor="k", zorder=6)

        ly = y0 - 1.5 if s.inward[1] > 0 else y1 + 1.5
        ax.text(
            s.door_center[0] + 1.5, ly,
            f"{s.name}\nFOV area: {s.visible_area_m2:.1f} m² (range {s.min_range:.1f}–{min(cfg.sensor.radius, s.max_range):.1f} m)",
            fontsize=8.5, zorder=6, va="center", ha="left",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=color, lw=1.2, alpha=0.9),
        )
        print(f"{s.name:16s} | camera=({s.camera[0]:5.2f}, {s.camera[1]:5.2f}) m | "
              f"ground range={s.min_range:.2f}..{min(cfg.sensor.radius, s.max_range):.2f} m | "
              f"visible floor area={s.visible_area_m2:5.2f} m2")

    sc = cfg.sensor
    ax.set_title(
        f"Floor Plan & Camera FOV Preview ({W:.2f} m × {H:.2f} m)  |  "
        f"HFOV={sc.fov_deg}°, VFOV={sc.vfov_deg}°, height={sc.mount_height} m, "
        f"tilt={sc.tilt_deg}°, max radius={sc.radius} m"
    )
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=args.dpi)
        print(f"\nSaved preview image to: {os.path.abspath(args.save)}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
