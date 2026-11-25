import json
from pathlib import Path
import unittest

import numpy as np

from sfm.config import Config, ExitConfig, MapConfig
from sfm.crowd import Crowd, pair_force_magnitudes
from sfm.simulation import Simulation


class CounterflowFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).parent / 'fixtures/counterflow_350_70-30_t64.json'
        cls.snapshot = json.loads(path.read_text(encoding='utf-8'))
        cls.cfg = Config()
        cls.cfg.map = MapConfig(**cls.snapshot['map'])
        cls.cfg.map.image_path = str(path.with_name(cls.cfg.map.image_path))
        cls.cfg.exits = [ExitConfig(**data) for data in cls.snapshot['exits']]
        cls.geometry = Simulation(cls.cfg, initial_positions=np.empty((0, 2)), verbose=False)
        cls.rooms = cls.geometry.floor.detect_rooms(cls.cfg.spawn.door_close_radius_px)

    def crowd(self, selected):
        snap = self.snapshot
        group = Crowd(np.array(snap['pos'])[selected], np.array(snap['v0'])[selected],
                      self.cfg.ped, self.cfg.force, np.random.default_rng(0), self.geometry.nav)
        group.vel[:] = np.array(snap['vel'])[selected]
        group.target[:] = np.array(snap['target'])[selected]
        group.ids[:] = np.array(snap['ids'])[selected]
        group.counterflow_rooms = self.rooms
        return group

    def test_two_way_drift_and_one_way_control_for_replayed_snapshot(self):
        self.assertEqual(self.geometry.floor.image_sha256, self.snapshot['map_sha256'])
        target = np.array(self.snapshot['target'])
        results = []
        for dt in (.005, .01, .02):
            for two_way in (False, True):
                with self.subTest(dt=dt, two_way=two_way):
                    group = self.crowd(np.ones(len(target), dtype=bool) if two_way else target == 2)
                    tracked = np.isin(group.ids, [241, 287])
                    nav = self.geometry.nav[2]
                    before = nav.distance(group.pos[tracked])
                    start = group.pos[tracked, 0].copy()
                    self.assertTrue((group.desired_directions()[tracked, 0] < -.9).all())
                    for _ in range(round(2 / dt)):
                        group.step(self.geometry.floor, dt)
                        self.assertTrue((self.geometry.floor.wall_distance(group.pos) >= group.radius).all())
                        self.assertEqual(float(group.contact[tracked].max()), 0.)
                    displacement = group.pos[tracked, 0] - start
                    change_in_route = nav.distance(group.pos[tracked]) - before
                    if two_way:
                        self.assertTrue((displacement > .8).all())
                        self.assertTrue((change_in_route > .75).all())
                        results.append(displacement)
                    else:
                        self.assertTrue((displacement < -2.).all())
                        self.assertTrue((change_in_route < -2.).all())
        self.assertLess(np.ptp(np.array(results), axis=0).max(), .02)

    def test_social_repulsion_body_and_friction_at_fixture_and_contact(self):
        s = self.snapshot
        x = np.array(s['pos'])
        a, b = (s['ids'].index(i) for i in (287, 241))
        delta = x[a] - x[b]
        distance = np.linalg.norm(delta)
        normal = delta / distance
        tangent = np.array([-normal[1], normal[0]])
        v = np.array(s['vel'])
        social, body, friction = pair_force_magnitudes(
            self.cfg.force, 2 * self.cfg.ped.radius - distance, (v[b] - v[a]) @ tangent)
        self.assertGreater(social * normal[0], 300.)
        self.assertEqual((body, friction), (0., 0.))
        social, body, friction = pair_force_magnitudes(self.cfg.force, .01, -.25)
        self.assertGreater(social, self.cfg.force.A)
        self.assertAlmostEqual(body, 1200.)
        self.assertAlmostEqual(friction, -600.)
