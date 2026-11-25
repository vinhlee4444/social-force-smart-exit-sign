"""The geometry used by the saved head-on and classroom snapshots."""
from pathlib import Path

from sfm.config import Config, ExitConfig


def legacy_config():
    cfg = Config()
    cfg.map.image_path = str(Path(__file__).parent.resolve() / "fixtures/old_floor_6cb4552.png")
    cfg.map.seal_rects = []
    cfg.map.spawn_split_px = 691
    cfg.map.exit_door_width_m = 2.4
    cfg.exits = [
        ExitConfig("E1 top-left", (295, 1, 366, 98), (307, 99, 354, 100), (0, -1)),
        ExitConfig("E2 top-right", (1106, 1, 1177, 98), (1118, 99, 1165, 100), (0, -1)),
        ExitConfig("E3 bottom-left", (295, 630, 367, 727), (308, 628, 354, 629), (0, 1)),
        ExitConfig("E4 bottom-right", (1106, 630, 1177, 727), (1118, 628, 1164, 629), (0, 1)),
    ]
    return cfg
