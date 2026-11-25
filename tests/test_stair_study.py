import copy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import cv2

import numpy as np

from scenarios import BASE, SCENARIOS, initial_positions, spawn_layout
from sfm.config import Config, ExitConfig, GateConfig
from sfm.exit_sign import ExitSign, GateController
from sfm.render import ComparisonPanel, save_comparison
from sfm.simulation import Simulation
from legacy_map import legacy_config


class SignalTests(unittest.TestCase):
    def test_density_boundaries_and_dwell(self):
        gate = GateController(GateConfig())
        gate.update(0, np.nan)
        self.assertEqual(gate.state, "PENDING")
        gate.update(0, 3)
        self.assertEqual(gate.state, "GREEN")
        gate.update(5, 3)
        self.assertEqual(gate.state, "GREEN")
        gate.update(6, 3.001)
        self.assertEqual(gate.state, "RED")
        gate.update(15, 1)
        self.assertEqual(gate.state, "RED")
        gate.update(16, 1.001)
        self.assertEqual(gate.state, "RED")
        gate.update(17, 1)
        self.assertEqual(gate.state, "GREEN")
        gate.update(22, 2)
        self.assertEqual(gate.state, "GREEN")
        initially_red = GateController(GateConfig())
        initially_red.update(0, 3.01)
        self.assertEqual(initially_red.state, "RED")


class StudyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = legacy_config()
        cls.geometry = Simulation(cfg, BASE, initial_positions=np.empty((0, 2)), verbose=False)
        # A clear hall exercises switching independently of the study map's walls.
        cfg = copy.deepcopy(cfg)
        with TemporaryDirectory() as directory:
            path = Path(directory) / "hall.png"
            image = np.full((280, 600), 255, np.uint8)
            image[[0, -1], :] = image[:, [0, -1]] = 0
            cv2.imwrite(str(path), image)
            cfg.map.image_path = str(path)
            cfg.exits = [
                ExitConfig("E1", (115, 1, 184, 78), (126, 79, 173, 80), (0, -1)),
                ExitConfig("E2", (415, 1, 484, 78), (426, 79, 473, 80), (0, -1)),
            ]
            cls.hall = Simulation(cfg, directory, initial_positions=np.empty((0, 2)), verbose=False)
            occluded = copy.deepcopy(cfg)
            occluded.map.image_path = str(path.with_name("occluded.png"))
            image[75:161, 360] = 0
            cv2.imwrite(occluded.map.image_path, image)
            cls.occluded_hall = Simulation(occluded, directory, initial_positions=np.empty((0, 2)),
                                           verbose=False)

    def simulation(self, pos, p=1, enabled=True, geometry=None, duration=120):
        g = geometry or self.geometry
        cfg = copy.deepcopy(g.cfg)
        cfg.sim.t_max = duration
        cfg.gate.enabled = enabled
        cfg.gate.min_open_time = cfg.gate.min_closed_time = 0
        cfg.guidance.switch_probability = p
        cfg.guidance.reaction_delay = (2, 2)
        return Simulation(cfg, BASE, g.floor, g.nav, np.asarray([pos]), verbose=False,
                          camera_masks=[s.mask for s in g.signs])

    def publish_signs(self, sim, t):
        sim.guidance.network.publish(t, (sign.state for sign in sim.signs))

    def test_camera_count_and_units(self):
        sign = self.geometry.signs[0]
        y, x = np.nonzero(sign.mask)
        person = (np.array([x[0], y[0]]) + .5) * sign.floor.s
        behind = sign.door_center + sign.inward * 2
        self.assertEqual(sign.count(np.array([person, behind, [-1, -1]])), 1)
        self.assertLess(sign.effective_range, 11.2)
        sensor = copy.deepcopy(sign.sensor)
        sensor.smooth_window = 0
        camera = ExitSign(0, sign.cfg, sign.floor, sensor, GateConfig(), 1, mask=sign.mask)
        camera.update(0, 0)
        self.assertEqual(camera.state, "GREEN")
        camera.update(5, 8)
        self.assertAlmostEqual(camera.last_value, 8 / sign.visible_area_m2)
        self.assertEqual(camera.state, "RED")
        with self.assertRaisesRegex(ValueError, "zero floor area"):
            ExitSign(0, sign.cfg, sign.floor, sensor, GateConfig(), 1, mask=~np.ones_like(sign.mask))

    def test_pair_hash_and_quota_nonoverlap(self):
        slots, _ = spawn_layout(self.geometry)
        floor = self.geometry.floor
        rooms = floor.detect_rooms(self.geometry.cfg.spawn.door_close_radius_px)
        for quota in SCENARIOS.values():
            if any(n > len(side) for n, side in zip(quota, slots)):
                continue
            pos = initial_positions(slots, quota, 0)
            distances = np.linalg.norm(pos[:, None] - pos[None, :], axis=2)
            np.fill_diagonal(distances, np.inf)
            self.assertGreaterEqual(distances.min(), 1.05 - 1e-9)
            self.assertGreaterEqual(floor.wall_distance(pos).min(), .6)
            self.assertEqual((pos[:, 0] < floor.width_m / 2).sum(), quota[0])
            self.assertTrue((rooms[tuple(reversed(floor.to_px(pos)))] > 0).all())
        off = self.simulation([9.0, 6.1], enabled=False)
        on = self.simulation([9.0, 6.1])
        self.assertEqual(off.initial_state_hash(), on.initial_state_hash())

    def test_pass_through_red_and_commit(self):
        sim = self.simulation([16.55, 6.1], duration=8)
        sim.cfg.gate.min_closed_time = 10
        sim.signs[0].gate.update(0, 4)
        self.publish_signs(sim, 0)
        self.assertEqual(sim.signs[0].state, "RED")
        summary = sim.run(progress=False)
        self.assertEqual(summary["status"], "COMPLETED")
        self.assertEqual(sim.evac_exit[0], 0)
        self.assertEqual(sim.signs[0].state, "RED")
        self.assertEqual(summary["switches"], 0)
        self.assertTrue(sim.guidance.committed[0])

    def test_single_agent_crosses_body_clear_room_door(self):
        # This spawn balanced the driving force against the upstream jamb
        # gradient and stood still indefinitely before reaching the hall.
        for pos in ([38.61, 33.95], [39.61, 3.2], [44.86, 20.97], [28.13, 16.19]):
            with self.subTest(pos=pos):
                sim = self.simulation(pos, enabled=False, duration=30)
                self.assertEqual(sim.run(progress=False)["status"], "COMPLETED")

    def test_wall_adjacent_route_retains_tangential_approach(self):
        # Pressing a body against the wall beside the hall door used to
        # replace its route with a vector back into the crowd.
        nav = self.geometry.nav[1]
        directions = nav.direction(np.array([[59.45, 7.15], [59.55, 7.15]]))
        self.assertLess(directions[0, 1], -0.6)
        np.testing.assert_allclose(directions[0], directions[1], atol=1e-8)

    def test_room28_snapshot_clears_when_followers_yield_at_the_jamb(self):
        fixture = Path(__file__).parent / "fixtures" / "room28_t150.json"
        occupants = json.loads(fixture.read_text(encoding="utf-8"))
        geometry = self.geometry
        positions = np.array([entry["pos"] for entry in occupants])
        rooms = geometry.floor.detect_rooms(geometry.cfg.spawn.door_close_radius_px)
        self.assertTrue((rooms[tuple(reversed(geometry.floor.to_px(positions)))] == 28).all())
        for yielding, deadline in ((False, 3), (True, 120)):
            with self.subTest(yielding=yielding):
                cfg = copy.deepcopy(geometry.cfg)
                cfg.gate.enabled = False
                cfg.room_flow.enabled = yielding
                cfg.sim.t_max = deadline
                sim = Simulation(cfg, BASE, geometry.floor, geometry.nav, positions, verbose=False,
                                 camera_masks=[s.mask for s in geometry.signs])
                for name in ("v0", "vel", "target"):
                    setattr(sim.crowd, name, np.array([entry[name] for entry in occupants]))
                result = sim.run(progress=False)
                if yielding:
                    self.assertEqual(result["status"], "COMPLETED")
                else:
                    self.assertEqual(result["remaining"], len(occupants))
                    np.testing.assert_allclose(sim.crowd.pos, positions, atol=0.02, rtol=0)

    def test_probability_delay_cooldown_and_target_persistence(self):
        off = self.simulation([13, 8], p=0, geometry=self.hall)
        off.signs[0].gate.update(0, 4)
        self.publish_signs(off, 0)
        off.guidance.update(0)
        off.guidance.update(2)
        self.assertEqual(off.crowd.target[0], 0)

        sim = self.simulation([13, 8], geometry=self.hall)
        guide = sim.guidance
        sim.signs[0].gate.update(0, 4)
        self.publish_signs(sim, 0)
        guide.update(0)
        guide.update(1.99)
        self.assertEqual(sim.crowd.target[0], 0)
        guide.update(2)
        changed = int(sim.crowd.target[0])
        self.assertEqual(changed, 1)
        sim.signs[changed].gate.update(3, 4)
        sim.signs[0].gate.update(3, 0)
        self.publish_signs(sim, 3)
        sim.step()
        self.assertEqual(sim.crowd.target[0], changed)
        guide.update(7)
        self.assertFalse(np.isfinite(guide.react_at[0]))
        guide.update(12)
        guide.update(14)
        self.assertEqual(sim.crowd.target[0], 0)
        self.assertEqual(len(guide.events), 2)

    def test_step_switches_when_reader_remains_eligible(self):
        sim = self.simulation([13, 9.2], p=1, geometry=self.hall, duration=5)
        sim.cfg.guidance.reaction_delay = (1, 1)
        sim.cfg.gate.min_closed_time = 10
        # Supply a controlled red sample in the test hall.
        sim.signs[0].gate.update(0, 4)
        self.publish_signs(sim, 0)
        self.assertEqual([sign.state for sign in sim.signs], ["RED", "GREEN"])
        self.assertEqual(int(sim.crowd.target[0]), 0)
        for _ in range(100):
            sim.step()
        self.assertEqual(int(sim.crowd.target[0]), 0)
        self.assertEqual(sim.guidance.events, [])
        self.assertAlmostEqual(sim.guidance.react_at[0], sim.t)
        self.assertFalse(sim.guidance.committed[0])
        positions = sim.crowd.pos
        self.assertGreater(sim.signs[0].distance_to_door(positions)[0],
                           sim.cfg.guidance.commit_distance)
        self.assertTrue(all(sim.guidance._readable(sign.camera, positions)[0] for sign in sim.signs))
        current, alternative = [nav.distance(positions)[0] for nav in sim.nav]
        self.assertTrue(np.isfinite([current, alternative]).all())
        self.assertGreater(alternative, current)
        self.assertLessEqual(alternative - current, sim.cfg.guidance.max_extra_distance)
        sim.step()
        self.assertEqual(int(sim.crowd.target[0]), 1)
        self.assertEqual(sim.guidance.events, [(1.0, 0, 0, 1)])
        for _ in range(50):
            sim.step()
            self.assertEqual(int(sim.crowd.target[0]), 1)
        self.assertEqual(len(sim.guidance.events), 1)
        panel = ComparisonPanel(sim, "ON", (1, 0))
        with patch.object(panel, "write", wraps=panel.write) as write:
            panel.frame()
        self.assertIn("ID 0  E1 -> E2", [call.args[1] for call in write.call_args_list])

    def test_reaction_rechecks_transmitted_signal(self):
        sim = self.simulation([13, 8], geometry=self.hall)
        guide = sim.guidance
        sim.signs[0].gate.update(0, 4)
        self.publish_signs(sim, 0)
        guide.update(0)
        self.assertTrue(np.isfinite(guide.react_at[0]))
        sim.signs[0].gate.update(1, 0)
        self.publish_signs(sim, 1)
        guide.update(2)
        self.assertEqual(sim.crowd.target[0], 0)

    def test_readable_red_sign_recommends_hidden_green_door(self):
        sim = self.simulation([13, 8], p=1, geometry=self.occluded_hall)
        guide = sim.guidance
        # The routing fields include the wall that separates the signs.
        self.assertFalse(sim.floor.visible_from(sim.signs[0].camera, sim.signs[1].camera[None])[0])
        self.assertTrue(guide._readable(sim.signs[0].camera, sim.crowd.pos)[0])
        self.assertFalse(guide._readable(sim.signs[1].camera, sim.crowd.pos)[0])
        sim.signs[0].gate.update(0, 4)
        guide.update(0)
        self.assertFalse(np.isfinite(guide.react_at[0]))
        self.publish_signs(sim, 5)
        guide.update(4.99)
        self.assertFalse(np.isfinite(guide.react_at[0]))
        guide.update(5)
        self.assertAlmostEqual(guide.react_at[0], 7)
        guide.update(7)
        self.assertEqual(guide.events, [(7, 0, 0, 1)])
        decision = guide.decisions[-1]
        self.assertEqual((decision["id"], decision["source"], decision["old_exit"],
                          decision["new_exit"], decision["reason"]), (0, "E1", "E1", "E2", "switched"))

    def test_hidden_source_sign_prevents_switch(self):
        sim = self.simulation([13, 8], p=1, geometry=self.hall)
        sim.floor = copy.copy(sim.floor)
        sim.floor.base_free = sim.floor.base_free.copy()
        sim.floor.base_free[75:161, 210] = False
        sim.signs[0].gate.update(0, 4)
        self.publish_signs(sim, 0)
        sim.guidance.update(0)
        self.assertFalse(np.isfinite(sim.guidance.react_at[0]))
        self.assertEqual(sim.guidance.decisions[-1]["reason"], "source_sign_unreadable")

    def test_invalid_summary_preserves_old_video(self):
        off = self.simulation([9, 6.1], enabled=False, duration=0.1)
        on = self.simulation([9, 6.1], duration=0.1)
        with TemporaryDirectory() as directory:
            video = Path(directory) / "OFF_vs_ON.mp4"
            video.write_bytes(b"old video")
            with patch("sfm.render._verify_end_card", side_effect=RuntimeError("bad card")):
                with self.assertRaisesRegex(RuntimeError, "bad card"):
                    save_comparison(off, on, video, (1, 0))
            self.assertEqual(video.read_bytes(), b"old video")
            self.assertEqual(list(video.parent.iterdir()), [video])



if __name__ == "__main__":
    unittest.main()
