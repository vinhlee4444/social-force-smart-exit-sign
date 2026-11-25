import copy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np

from sfm.config import Config
from sfm.crowd import Crowd
from sfm.floorplan import FloorPlan
from sfm.route_advice import CoveredRoutes
from sfm.simulation import Simulation
from legacy_map import legacy_config


class HeadOnTests(unittest.TestCase):
    def test_snapshot_pair_and_step_size(self):
        cfg = legacy_config()
        sim = Simulation(cfg, initial_positions=np.empty((0, 2)), verbose=False)
        snapshot = json.loads((Path(__file__).parent / "fixtures/head_on_t300.json").read_text())
        durations = []
        for dt in (0.005, 0.01, 0.02):
            c = Crowd(snapshot["pos"], snapshot["v0"], cfg.ped, cfg.force,
                      np.random.default_rng(0), sim.nav)
            c.vel = np.asarray(snapshot["vel"])
            c.target = np.asarray(snapshot["target"])
            for step in range(round(30 / dt)):
                c.step(sim.floor, dt)
                self.assertTrue((sim.floor.wall_distance(c.pos) >= c.radius).all())
                reached = np.logical_or.reduce([sign.evacuated(c.pos) for sign in sim.signs])
                if reached.any():
                    c.remove(reached)
                if not len(c):
                    durations.append((step + 1) * dt)
                    break
            self.assertEqual(len(c), 0)
        self.assertLess(max(durations) - min(durations), 0.1)
        original = copy.deepcopy(cfg.ped)
        original.avoid_head_on = False
        c = Crowd(snapshot["pos"], snapshot["v0"], original, cfg.force,
                  np.random.default_rng(0), sim.nav)
        c.vel = np.asarray(snapshot["vel"])
        c.target = np.asarray(snapshot["target"])
        for _ in range(3000):
            c.step(sim.floor, 0.01)
        self.assertLess(np.linalg.norm(c.pos - snapshot["pos"], axis=1).max(), 1.0)

    def test_two_and_six_walkers_in_corridors(self):
        cfg = Config()
        cfg.map.seal_rects = []
        with TemporaryDirectory() as temp:
            path = Path(temp) / "floor.png"
            cfg.map.image_path = str(path)
            for width, n in ((2.45, 1), (2.45, 3), (3.8, 3), (2.10, 1), (1.30, 1)):
                with self.subTest(width=width, walkers=2 * n):
                    image = np.zeros((round(width / 0.05) + 2, 400), np.uint8)
                    image[1:-1, 1:-1] = 255
                    cv2.imwrite(str(path), image)
                    floor = FloorPlan(cfg.map, [])
                    y = floor.height_m / 2
                    pos = [[8 - i * 1.45, y] for i in range(n)] + [[10 + i * 1.45, y] for i in range(n)]
                    directions = np.array([[1, 0]] * n + [[-1, 0]] * n, dtype=float)
                    c = Crowd(pos, np.full(2 * n, 1.6), cfg.ped, cfg.force, np.random.default_rng(1))
                    for _ in range(1200):
                        c.step(floor, 0.01, directions[c.ids])
                        self.assertTrue((floor.wall_distance(c.pos) >= c.radius).all())
                        passed = np.where(c.ids < n, c.pos[:, 0] > 12, c.pos[:, 0] < 6)
                        if passed.any():
                            c.remove(passed)
                        if not len(c):
                            break
                    if width >= 2.45:
                        self.assertEqual(len(c), 0)
                    elif width == 1.30:
                        self.assertEqual(len(c), 2)

    def test_coverage_and_conflicting_advertisements(self):
        # Isolate the hub's information rules from path construction.
        rules = CoveredRoutes.__new__(CoveredRoutes)
        rules.coverage = [0.25, 1.0]
        rules.advertised = set()
        a, b = (10, 3), (11, 3)
        rules.routes = {(0, 1): {(a, b)}, (1, 0): {(b, a)}}
        self.assertEqual(rules.filter(1, (0,)), ((), "approach_unobserved"))
        self.assertEqual(rules.filter(0, (1,))[0], (1,))
        rules.reserve(0, 1)
        rules.coverage[0] = 1.0
        self.assertEqual(rules.filter(1, (0,)), ((), "unmapped_or_opposing_advertisement"))

    def test_west_corridor_direction_and_real_coverage(self):
        sim = Simulation(legacy_config(), initial_positions=np.empty((0, 2)), verbose=False)
        rules = CoveredRoutes(sim)
        self.assertTrue(all(0 < portion < 1 for portion in rules.coverage))
        self.assertEqual(rules.filter(0, (2,)), ((), "approach_unobserved"))
        # Hold coverage constant solely to exercise the route conflict rule.
        rules.coverage = [1.0] * len(sim.signs)
        self.assertEqual(rules.filter(0, (2,))[0], (2,))
        rules.reserve(0, 2)
        self.assertEqual(rules.filter(2, (0,)), ((), "unmapped_or_opposing_advertisement"))
