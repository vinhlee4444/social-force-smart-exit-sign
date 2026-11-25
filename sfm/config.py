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
If `1.jpg` is edited, walls are re-detected automatically and spawn rooms are
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
    image_path: str = "1.jpg"
    # Existing door spans remain 23–25 px after cropping the layout.
    # Keep the 0.05 m/px door-width reference until a surveyed dimension is supplied.
    meters_per_pixel: float = 0.05
    wall_threshold: int = 127              # grey value <= threshold is wall
    # Rectangles painted as wall on load, to repair holes in the drawn map.
    # The JPEG cuts off the former hall beyond the east wall. Exclude its
    # truncated strip and close the two openings into the omitted space.
    seal_rects: List[Rect] = field(default_factory=lambda: [(1179, 0, 1187, 729), (0, 729, 1187, 729)])
    spawn_split_px: Optional[float] = None  # default: midpoint of usable floor extent
    # Optional width cap in metres. The revised map uses the measured raster gaps.
    exit_door_width_m: Optional[float] = None


@dataclass
class ExitConfig:
    """An always-passable door leading to a destination zone on this floor."""
    name: str
    room: Rect                  # interior of the exit room (stairwell), px
    door: Rect                  # physical door gap, px
    inward: Tuple[int, int]     # unit direction corridor -> room, image axes ((0,-1) = up)


def default_exits() -> List[ExitConfig]:
    # Re-measured on 1.jpg (1188 x 730), inheriting the identified exit rooms.
    return [
        ExitConfig("E1 top-left",     room=(295, 1, 366, 98),     door=(307, 99, 354, 100),   inward=(0, -1)),
        ExitConfig("E2 top-right",    room=(1107, 1, 1178, 98),   door=(1119, 99, 1166, 100), inward=(0, -1)),
        ExitConfig("E3 bottom-left",  room=(296, 632, 367, 728),  door=(308, 630, 355, 631),  inward=(0, 1)),
        ExitConfig("E4 bottom-right", room=(1107, 632, 1178, 728), door=(1119, 630, 1165, 631), inward=(0, 1)),
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
    avoid_head_on: bool = True             # steer right at a projected opposing encounter


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
    cell_size: float = 0.1                 # m, resolution of the route grid
    wall_clearance: float = 1.0            # m, paths prefer staying this far from walls
    wall_penalty: float = 4.0              # extra cost factor right next to a wall
    target_depth: float = 1.5              # m, route target = exit-room cells this deep past the door
    evac_depth: float = 1.0                # m, an agent this deep inside an exit room has evacuated
    cache_dir: str = "cache"
    rebuild_cache: bool = False           # skip reading the cache for geometry verification


@dataclass
class SensorConfig:
    """Simulated corridor-facing camera with illustrative lens and mounting geometry."""
    radius: float = 11.2                   # m, configured upper limit
    fov_deg: float = 58.6                  # horizontal field of view; 360 = omnidirectional (old behaviour)
    line_of_sight: bool = True             # cameras cannot see through walls
    # An AI crowd counter (CSRNet / DM-Count) on an edge device needs a fraction of a second
    # to a few seconds per frame, so the sign does NOT read the crowd 10x per second.
    period: float = 1.0                    # s between two camera readings
    count_error: float = 0.0               # exact simulated count by default
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
    enabled: bool = True                   # display congestion signals
    close_above: float = 3.0               # RED above this density (people/m²)
    reopen_at_most: float = 1.0            # GREEN at or below this density
    min_closed_time: float = 10.0          # minimum RED dwell time, s
    min_open_time: float = 5.0             # minimum GREEN dwell time, s


@dataclass
class GuidanceConfig:
    policy: str = "congestion_guidance"     # or "off"
    switch_probability: float = 0.6
    reaction_delay: Tuple[float, float] = (1.0, 3.0)
    commit_distance: float = 5.0            # m to the door face
    switch_cooldown: float = 10.0
    decision_recheck_period: float = 5.0
    max_extra_distance: float = 30.0
    read_distance: float = 16.0             # m, illustrative sign legibility
    approach_length: float = 10.0           # m of approach requiring evidence in covered_routes
    branch_min_shared_m: float = 1.0       # m of mapped path still shared ahead
    branch_lookahead_m: float = 8.0        # m to look for the split
    branch_extra_ratio: float = 0.5        # maximum extra distance relative to remaining original route
    opposing_corridor_width_m: float = 2.0 # m between opposite mapped centre paths, with clear floor


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
class RoomFlowConfig:
    enabled: bool = True
    decision_period: float = 0.1           # s, independent of video sampling
    stall_speed: float = 0.05              # m/s, all occupants must remain below this speed
    stall_duration: float = 2.0            # s of collective standstill before yielding


@dataclass
class Config:
    map: MapConfig = field(default_factory=MapConfig)
    exits: List[ExitConfig] = field(default_factory=default_exits)
    ped: PedestrianConfig = field(default_factory=PedestrianConfig)
    force: ForceConfig = field(default_factory=ForceConfig)
    nav: NavConfig = field(default_factory=NavConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    gate: GateConfig = field(default_factory=GateConfig)
    guidance: GuidanceConfig = field(default_factory=GuidanceConfig)
    spawn: SpawnConfig = field(default_factory=SpawnConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    room_flow: RoomFlowConfig = field(default_factory=RoomFlowConfig)

