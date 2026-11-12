"""
All simulation parameters live here. No other module should contain "magic numbers".

Conventions
-----------
* Physics runs in SI units: metres, seconds, kilograms, newtons.
* Pixel coordinates are only used to describe things drawn on the floor-plan image
  (exits, seal rectangles). They are converted with `MapConfig.meters_per_pixel`.
* World origin = top-left corner of the image, x to the right, y DOWN (same as the image).
* A pixel rectangle `Rect` is (x0, y0, x1, y1), inclusive, in image pixel indices.

Changing the map
----------------
If `1.png` is edited, walls are re-detected automatically and spawn rooms are
re-detected automatically (see `SpawnConfig`). You only need to update:
  1. `MapConfig.image_path` / `meters_per_pixel` if the file or scale changed,
  2. `ExitConfig` entries (room + door rectangles) so they match the new exits,
  3. `MapConfig.seal_rects` if the new image has holes in its outer border.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Rect = Tuple[int, int, int, int]  # (x0, y0, x1, y1) inclusive, image pixels


@dataclass
class MapConfig:
    image_path: str = "1.png"
    meters_per_pixel: float = 0.05         # 1 px = 0.05 m -> classroom doors are 1.2m wide, exit doors are 2.4m wide 
    wall_threshold: int = 127              # grey value <= threshold is wall
    # Rectangles painted as wall on load, to repair holes in the drawn map.
    seal_rects: List[Rect] = field(default_factory=lambda: [])
    # Narrow every exit door to this clear width (m), centred on the drawn gap. None = as drawn
    # (4.8 m, wider than a real exit door; real doors are ~1.2-2.4 m).
    exit_door_width_m: Optional[float] = 2.4


@dataclass
class ExitConfig:
    """One exit = one exit sign + one gate. The exit room acts as the 'outside'."""
    name: str
    room: Rect                  # interior of the exit room (stairwell), px
    door: Rect                  # door gap that the gate blocks when closed, px
    inward: Tuple[int, int]     # unit direction corridor -> room, image axes ((0,-1) = up)


def default_exits() -> List[ExitConfig]:
    # Measured from 1.png (1382 x 729).
    return [
        ExitConfig("E1 top-left",     room=(295, 1, 366, 98),     door=(307, 99, 354, 100),   inward=(0, -1)),
        ExitConfig("E2 top-right",    room=(1106, 1, 1177, 98),   door=(1118, 99, 1165, 100), inward=(0, -1)),
        ExitConfig("E3 bottom-left",  room=(295, 630, 367, 727),  door=(308, 628, 354, 629),  inward=(0, 1)),
        ExitConfig("E4 bottom-right", room=(1106, 630, 1177, 727), door=(1118, 628, 1164, 629), inward=(0, 1)),
    ]


@dataclass
class PedestrianConfig:
    """Identical body parameters for every agent; only the desired speed varies."""
    mass: float = 80.0                     # kg
    radius: float = 0.5                    # m (shoulder half-width)
    v0_min: float = 1.5                    # desired speed ~ U(v0_min, v0_max), hurried evacuation
    v0_max: float = 2.0
    tau: float = 0.5                       # s, relaxation time
    v_max_factor: float = 1.3              # speed is capped at v_max_factor * v0


@dataclass
class ForceConfig:
    """Helbing, Farkas & Vicsek (2000) parameters."""
    A: float = 2000.0                      # N,  agent-agent social repulsion strength
    B: float = 0.08                        # m,  agent-agent repulsion range
    A_wall: float = 2000.0                 # N
    B_wall: float = 0.08                   # m
    k: float = 1.2e5                       # kg/s^2,     body compression
    kappa: float = 2.4e5                   # kg/(m s),   sliding friction
    cutoff: float = 2.0                    # m, ignore agent pairs further apart than this
    skin: float = 0.5                      # m, Verlet neighbour-list margin
    noise_std: float = 0.0                 # N, optional random fluctuation force


@dataclass
class NavConfig:
    """Navigation (route) fields: geodesic distance to each exit."""
    cell_size: float = 0.2                 # m, resolution of the route grid
    wall_clearance: float = 1.0            # m, paths prefer staying this far from walls
    wall_penalty: float = 4.0              # extra cost factor right next to a wall
    target_depth: float = 1.5              # m, route target = exit-room cells this deep past the door
    evac_depth: float = 1.0                # m, an agent this deep inside an exit room has evacuated
    cache_dir: str = "cache"


@dataclass
class SensorConfig:
    """Smart exit sign camera (OmniVision OV5648: HFOV 58.6 deg, VFOV 45.3 deg).

    The camera sits above the door and looks into the corridor (opposite of `inward`),
    so its floor footprint is a CONE, not a circle.
    """
    radius: float = 8.0                    # m, max counting range (8-10 m; > 10 m is not trusted)
    fov_deg: float = 58.6                  # horizontal field of view; 360 = omnidirectional (old behaviour)
    line_of_sight: bool = True             # cameras cannot see through walls
    # An AI crowd counter (CSRNet / DM-Count) on an edge device needs a fraction of a second
    # to a few seconds per frame, so the sign does NOT read the crowd 10x per second.
    period: float = 1.0                    # s between two camera readings
    count_error: float = 0.10              # relative std of the AI count (0 = perfect counter)
    # --- mounting: camera looks DOWN from above the door -> blind zone right below it ---
    use_min_range: bool = True             # False = ignore mounting geometry (no blind zone)
    vfov_deg: float = 45.3                 # vertical field of view
    mount_height: float = 2.5              # m above floor (exit signs hang ~2.3-2.7 m)  <-- set the real value
    tilt_deg: float = 45.0                 # deg below horizontal                          <-- set the real value
    # --- what the gate controller reads ---
    metric: str = "density"                # "density" (people / m2 of visible floor) or "count"
    smooth_window: float = 3.0             # s, moving average over the readings (0 = no smoothing)


@dataclass
class GateConfig:
    enabled: bool = True                   # False = baseline (gates always open)
    # Thresholds are in the unit of SensorConfig.metric:
    #   density -> people / m2 of the camera's visible floor (~26 m2). 0.7 /m2 ~ 19 people in view.
    #   count   -> people
    close_above: float = 0.7               # "closed" when value > close_above
    reopen_at_most: float = 0.4            # reopen when value <= reopen_at_most ...
    min_closed_time: float = 10.0          # ... AND the gate has been closed >= this long
    max_closed_time: float = 20.0          # fail-safe: reopen after this long whatever the crowd does
    min_open_time: float = 5.0             # after (re)opening, stay open at least this long (anti-flicker)


@dataclass
class SpawnConfig:
    """Agents start inside rooms (not corridors, not exit rooms)."""
    n_agents: int = 300
    door_close_radius_px: int = 15         # morphological radius used to split rooms at their doors
    min_wall_gap: float = 0.1              # m, extra clearance between a spawned body and walls
    min_agent_gap: float = 0.05            # m, extra clearance between spawned bodies
    x_min_m: Optional[float] = None        # only spawn agents with x >= this (m); None = whole building
    x_max_m: Optional[float] = None        # only spawn agents with x <= this (m): crowd in one wing


@dataclass
class SimConfig:
    dt: float = 0.01                       # s
    t_max: float = 300.0                   # s, hard stop
    frame_period: float = 0.1              # s of sim time between rendered frames / records
    seed: int = 0
    danger_force_N: float = 1000.0         # an agent squeezed with more than this counts as "in danger"
                                           # (ASSUMPTION ~ 1.6 kN/m crowd-crush level x 0.6 m shoulders)


@dataclass
class Config:
    map: MapConfig = field(default_factory=MapConfig)
    exits: List[ExitConfig] = field(default_factory=default_exits)
    ped: PedestrianConfig = field(default_factory=PedestrianConfig)
    force: ForceConfig = field(default_factory=ForceConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    gate: GateConfig = field(default_factory=GateConfig)
    spawn: SpawnConfig = field(default_factory=SpawnConfig)
    sim: SimConfig = field(default_factory=SimConfig)

