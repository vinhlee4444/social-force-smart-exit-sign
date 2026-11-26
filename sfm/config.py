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
    meters_per_pixel: float = 0.05         # 1 px = 0.05 m  ->  1287 x 681 px = 64.35 x 34.05 m
    wall_threshold: int = 127              # grey value <= threshold is wall
    # Rectangles painted as wall on load, to repair holes in the drawn map.
    seal_rects: List[Rect] = field(default_factory=lambda: [])
    # Narrow every exit door to this clear width (m), centred on the drawn gap. None = as drawn
    # (1.75-1.80 m in 1.png; use e.g. 1.2 or 1.0 to test narrower bottlenecks).
    exit_door_width_m: Optional[float] = None


@dataclass
class ExitConfig:
    """One exit = one exit sign + one gate. The exit room acts as the 'outside'."""
    name: str
    room: Rect                  # interior of the exit room (stairwell), px
    door: Rect                  # door gap that the gate blocks when closed, px
    inward: Tuple[int, int]     # unit direction corridor -> room, image axes ((0,-1) = up)


def default_exits() -> List[ExitConfig]:
    # Measured from 1.png (1287 x 681).
    return [
        ExitConfig("E1 top-left",     room=(201, 2, 249, 75),     door=(206, 76, 241, 77),   inward=(0, -1)),
        ExitConfig("E2 top-right",    room=(952, 2, 1001, 75),    door=(958, 76, 992, 77),   inward=(0, -1)),
        ExitConfig("E3 bottom-left",  room=(201, 605, 249, 679),  door=(207, 603, 242, 604), inward=(0, 1)),
        ExitConfig("E4 bottom-right", room=(952, 605, 1001, 679), door=(960, 603, 995, 604), inward=(0, 1)),
    ]


@dataclass
class PedestrianConfig:
    """Identical body parameters for every agent; only the desired speed varies."""
    mass: float = 80.0                     # kg
    radius: float = 0.22                   # m (0.44 m circular diameter)
    v0_min: float = 1.8                    # desired speed ~ U(v0_min, v0_max), panic evacuation (m/s)
    v0_max: float = 2.5
    tau: float = 0.35                      # s, relaxation time (smaller tau = stronger panic pushing force)
    v_max_factor: float = 1.3              # speed is capped at v_max_factor * v0


@dataclass
class ForceConfig:
    """Helbing, Farkas & Vicsek (2000) parameters."""
    A: float = 600.0                       # N,  agent-agent social repulsion (reduced in panic so crowd packs & pushes)
    B: float = 0.04                        # m,  agent-agent repulsion falloff distance
    A_wall: float = 2000.0                 # N
    B_wall: float = 0.04                   # m,  wall repulsion range (allows passing 0.75 m room doors)
    k: float = 1.2e5                       # kg/s^2,     body compression
    kappa: float = 1.0e4                   # kg/(m s),   sliding friction (allows doorway arches to break naturally)
    cutoff: float = 2.0                    # m, ignore agent pairs further apart than this
    skin: float = 0.5                      # m, Verlet neighbour-list margin
    noise_std: float = 20.0                # N, random panic jostling fluctuation force


@dataclass
class NavConfig:
    """Navigation (route) fields: geodesic distance to each exit."""
    cell_size: float = 0.1                 # m, resolution of the route grid (2x2 px at 0.05 m/px)
    wall_clearance: float = 0.5            # m, paths prefer staying this far from walls
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
    radius: float = 11.8                   # m, max counting range (11.8m)
    fov_deg: float = 62.6                  # horizontal field of view; 360 = omnidirectional 
    line_of_sight: bool = True             # cameras cannot see through walls
    # An AI crowd counter (CSRNet / DM-Count) on an edge device needs a fraction of a second
    # to a few seconds per frame, so the sign does NOT read the crowd 10x per second.
    period: float = 1.0                    # s between two camera readings
    count_error: float = 0.10              # relative std of the AI count (0 = perfect counter)
    # --- mounting: camera looks DOWN from above the door -> blind zone right below it ---
    use_min_range: bool = True             # False = ignore mounting geometry (no blind zone)
    vfov_deg: float = 48                   # vertical field of view
    mount_height: float = 3                # m above floor (exit signs hang ~2.3-2.7 m)  <-- set the real value
    tilt_deg: float = 70.0                 # deg below horizontal                          <-- set the real value
    # --- what the gate controller reads ---
    metric: str = "density"                # "density" (people / m2 of visible floor) or "count"
    smooth_window: float = 3.0             # s, moving average over the readings (0 = no smoothing)


@dataclass
class GateConfig:
    enabled: bool = True                   # False = baseline (gates always open)
    # Thresholds are in the unit of SensorConfig.metric:
    #   density -> people / m2 of the camera's visible floor (~26 m2). 0.7 /m2 ~ 19 people in view.
    #   count   -> people
    close_above: float = 1.9               # "closed" when value > close_above
    reopen_at_most: float = 1.0            # reopen when value <= reopen_at_most ...
    min_closed_time: float = 4.0           # ... AND the gate has been closed >= this long
    max_closed_time: float = 20.0          # fail-safe: reopen after this long whatever the crowd does
    min_open_time: float = 5.0             # after (re)opening, stay open at least this long (anti-flicker)
    commit_radius: float = 5.0             # m, agents within this radius of a door exit through it despite a RED sign
    reaction_delay_min: float = 0.8        # s, min cognitive reaction delay after seeing a RED sign (Galea et al., 2014)
    reaction_delay_max: float = 2.0        # s, max cognitive reaction delay after seeing a RED sign
    current_door_preference: float = 0.30  # Pathfinder Current Door Preference hysteresis factor (prevents ping-ponging)


@dataclass
class SpawnConfig:
    """Agents start inside rooms (not corridors, not exit rooms)."""
    n_agents: int = 300
    door_close_radius_px: int = 10         # morphological radius used to split rooms at their doors (15-16 px wide)
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
    danger_force_N: float = 1000.0         # an agent squeezed with more than this experiences lethal-level crush force
                                           # (~1.6 kN/m crowd-crush threshold x ~0.6 m body breadth)
    danger_exposure_s: float = 3.0         # s, cumulative time above danger_force_N to count an agent as "in danger"


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

