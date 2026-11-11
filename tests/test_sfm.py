"""
Sanity tests. Run with:   python -m unittest discover -s tests -v
"""
import os
import sys
import unittest

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE)

from sfm.config import Config, GateConfig          # noqa: E402
from sfm.exit_sign import ExitSign, GateController   # noqa: E402
from sfm.floorplan import FloorPlan                  # noqa: E402
from sfm.navigation import build_navigation          # noqa: E402
from sfm.simulation import Simulation                # noqa: E402

_CACHE = {}


def shared_env():
    """Floor plan + route fields are expensive; build them once for all tests."""
    if not _CACHE:
        cfg = Config()
        floor = FloorPlan(cfg.map, cfg.exits, BASE)
        nav = build_navigation(floor, cfg.exits, cfg.nav, BASE, verbose=False)
        _CACHE["cfg"], _CACHE["floor"], _CACHE["nav"] = cfg, floor, nav
    # Fresh FloorPlan each time so gate states don't leak between tests.
    cfg = Config()
    return cfg, FloorPlan(cfg.map, cfg.exits, BASE), _CACHE["nav"]


class TestGateController(unittest.TestCase):
    def setUp(self):
        # Original two-state behaviour: no warning, no min-open time, no fail-safe.
        self.g = GateController(GateConfig(close_above=25, reopen_at_most=15, min_closed_time=10.0,
                                           close_warning_time=0.0, min_open_time=0.0,
                                           max_closed_time=float("inf")))

    def test_closes_only_above_threshold(self):
        self.assertFalse(self.g.update(0.0, 25))
        self.assertFalse(self.g.closed)
        self.assertTrue(self.g.update(0.1, 26))
        self.assertTrue(self.g.closed)

    def test_reopen_needs_both_conditions(self):
        self.g.update(0.0, 30)                       # closes at t = 0
        self.assertFalse(self.g.update(5.0, 0))      # empty, but only 5 s closed
        self.assertFalse(self.g.update(10.0, 16))    # 10 s closed, but 16 > 15 people
        self.assertTrue(self.g.update(10.1, 15))     # both conditions met
        self.assertFalse(self.g.closed)

    def test_disabled_gate_never_closes(self):
        g = GateController(GateConfig(enabled=False))
        self.assertFalse(g.update(0.0, 1000))
        self.assertFalse(g.closed)


class TestGateCases(unittest.TestCase):
    """The door cases added on top of the original two states."""

    def make(self, **kw):
        base = dict(close_above=1.0, reopen_at_most=0.6, min_closed_time=10.0, max_closed_time=20.0,
                    min_open_time=5.0, close_warning_time=3.0)
        base.update(kw)
        return GateController(GateConfig(**base))

    def test_warning_then_close_stays_passable_during_warning(self):
        g = self.make()
        self.assertFalse(g.update(10.0, 2.0))
        self.assertEqual(g.state, "CLOSING")
        self.assertTrue(g.passable and not g.closed)
        self.assertFalse(g.update(12.0, 2.0))        # warning not over yet
        self.assertTrue(g.update(13.0, 2.0))         # 3 s later, still crowded -> closes
        self.assertTrue(g.closed)

    def test_warning_cancelled_when_crowd_disperses(self):
        g = self.make()
        g.update(10.0, 2.0)
        self.assertFalse(g.update(13.0, 0.5))
        self.assertEqual(g.state, "OPEN")

    def test_forced_reopen_after_max_closed_time(self):
        g = self.make(close_warning_time=0.0, min_open_time=0.0)
        g.update(0.0, 5.0)
        self.assertFalse(g.update(15.0, 5.0))        # still jammed, not yet 20 s
        self.assertTrue(g.update(20.0, 5.0))         # fail-safe opens it anyway
        self.assertEqual(g.events[-1].reason, "max_closed_time")

    def test_min_open_time_prevents_flicker(self):
        g = self.make(close_warning_time=0.0, max_closed_time=float("inf"))
        g.update(0.0, 5.0)
        self.assertTrue(g.update(10.0, 0.0))         # reopens at t = 10
        self.assertFalse(g.update(12.0, 5.0))        # crowded again, but open only 2 s
        self.assertTrue(g.update(15.0, 5.0))         # 5 s open -> may close

    def test_last_open_exit_is_never_closed(self):
        g = self.make(close_warning_time=0.0, min_open_time=0.0)
        self.assertFalse(g.update(0.0, 5.0, can_close=False))
        self.assertFalse(g.closed)
        self.assertTrue(g.update(1.0, 5.0, can_close=True))


class TestMap(unittest.TestCase):
    def test_left_edge_sealed(self):
        cfg, floor, _ = shared_env()
        self.assertFalse(floor.free[:, 0].any(), "left border must be wall")

    def test_every_exit_reachable_from_corridors(self):
        cfg, floor, nav = shared_env()
        corridor_point = np.array([[60.0, 13.0]])     # top corridor, middle of the map
        for k, nf in enumerate(nav):
            self.assertTrue(np.isfinite(nf.distance(corridor_point))[0], cfg.exits[k].name)


class TestSensorCone(unittest.TestCase):
    def _sign(self, idx=2, gate=None, **overrides):
        cfg, floor, _ = shared_env()
        for k, v in overrides.items():
            setattr(cfg.sensor, k, v)
        for k, v in (gate or {}).items():
            setattr(cfg.gate, k, v)
        return cfg, floor, ExitSign(idx, cfg.exits[idx], floor, cfg.sensor, cfg.gate, cfg.nav.evac_depth)

    def test_legacy_circle_is_reproducible(self):
        # fov 360 + no blind zone + R=10 must give back the old circular zone (138.9 m2 for E3).
        _, _, s = self._sign(fov_deg=360.0, use_min_range=False, radius=10.0)
        self.assertAlmostEqual(s.visible_area_m2, 138.9, delta=0.5)

    def test_cone_faces_corridor_and_respects_limits(self):
        cfg, floor, s = self._sign()
        ys, xs = np.nonzero(s.mask)
        px, py = (xs + 0.5) * floor.s, (ys + 0.5) * floor.s
        v = np.stack([px - s.camera[0], py - s.camera[1]], 1)
        dist = np.linalg.norm(v, axis=1)
        cosang = (v @ s.facing) / dist
        self.assertGreaterEqual(cosang.min(), np.cos(np.radians(cfg.sensor.fov_deg / 2)) - 1e-6)
        self.assertLessEqual(dist.max(), cfg.sensor.radius + 0.1)
        self.assertGreaterEqual(dist.min(), s.min_range - 0.1)
        # E3's door is in the south wall of the corridor -> everything seen is NORTH of the camera.
        self.assertTrue((py < s.camera[1] + 1e-6).all())
        self.assertTrue(20.0 < s.visible_area_m2 < 35.0, s.visible_area_m2)

    def test_count_error_is_reproducible_and_unbiased(self):
        _, _, a = self._sign(count_error=0.10)
        _, _, b = self._sign(count_error=0.10)
        a.update(0.0, 20); b.update(0.0, 20)
        self.assertEqual(a.last_count, b.last_count)                 # same seed -> same reading
        readings = []
        for k in range(300):
            a.update(float(k), 20)
            readings.append(a.last_count)
        self.assertAlmostEqual(float(np.mean(readings)), 20.0, delta=0.7)

    def test_blind_zone_from_mounting_geometry(self):
        _, _, s = self._sign(mount_height=2.5, tilt_deg=20.0, vfov_deg=45.3)
        self.assertAlmostEqual(s.min_range, 2.5 / np.tan(np.radians(42.65)), places=3)
        self.assertEqual(s.max_range, float("inf"))                  # highest ray is above the horizon
        _, _, steep = self._sign(mount_height=2.5, tilt_deg=45.0)
        self.assertLess(steep.max_range, 10.0)                       # steeply tilted: far limit appears
        _, _, off = self._sign(use_min_range=False)
        self.assertEqual((off.min_range, off.max_range), (0.0, float("inf")))

    def test_density_and_smoothing(self):
        cfg, floor, s = self._sign(period=0.1, smooth_window=1.0, count_error=0.0, metric="density",
                                   gate=dict(close_above=0.2, reopen_at_most=0.1, close_warning_time=0.0,
                                             min_open_time=0.0))
        area = s.visible_area_m2
        for _ in range(10):
            s.update(0.0, 0)
        # a single-frame spike of 20 people is averaged away over the 1 s window ...
        self.assertFalse(s.update(1.0, 20))
        self.assertAlmostEqual(s.last_value, (20 / area) / 10, places=6)
        # ... a sustained crowd is not.
        closed = False
        for k in range(12):
            closed |= s.update(1.1 + 0.1 * k, 20)
        self.assertTrue(closed)
        self.assertTrue(s.closed)


class TestSimulation(unittest.TestCase):
    def test_single_agent_reaches_each_exit(self):
        cfg, floor, nav = shared_env()
        start = np.array([[60.0, 13.0]])
        for k in range(len(cfg.exits)):
            cfg.gate.enabled = False
            sim = Simulation(cfg, BASE, floor=floor, nav=nav, initial_positions=start, verbose=False)
            sim.crowd.target[:] = k
            sim.cfg.gate.reroute_on_close = False
            for _ in range(15000):                    # 150 s
                if sim.done:
                    break
                sim.step()
            self.assertEqual(sim.evac_exit[0], k, f"agent did not reach {cfg.exits[k].name}")

    def test_no_wall_penetration_and_deterministic(self):
        def run():
            cfg, floor, nav = shared_env()
            cfg.spawn.n_agents = 120
            cfg.sim.seed = 3
            sim = Simulation(cfg, BASE, floor=floor, nav=nav, verbose=False)
            worst = np.inf
            for _ in range(2000):                     # 20 s
                sim.step()
                if len(sim.crowd):
                    worst = min(worst, float(np.min(sim.floor.wall_distance(sim.crowd.pos))))
            return sim, worst

        sim_a, worst = run()
        self.assertGreater(worst, 0.0, f"an agent centre entered a wall (sdf={worst:.3f} m)")
        sim_b, _ = run()
        np.testing.assert_allclose(sim_a.crowd.pos, sim_b.crowd.pos)


class TestQueueEstimate(unittest.TestCase):
    def _signs(self, width):
        cfg = Config()
        cfg.map.exit_door_width_m = width
        floor = FloorPlan(cfg.map, cfg.exits, BASE)
        return cfg, [ExitSign(i, ex, floor, cfg.sensor, cfg.gate, cfg.nav.evac_depth)
                     for i, ex in enumerate(cfg.exits)]

    def test_door_width_follows_narrowing(self):
        _, narrow = self._signs(2.0)
        _, wide = self._signs(None)
        for n, w in zip(narrow, wide):
            self.assertAlmostEqual(n.door_width_m, 2.0, delta=0.2)
            self.assertGreater(w.door_width_m, 4.0)

    def test_estimated_wait_scales_with_people_and_width(self):
        cfg, narrow = self._signs(2.0)
        _, wide = self._signs(None)
        cfg.sensor.count_error = 0.0
        for s in (narrow[0], wide[0]):
            s.sensor.count_error = 0.0
            s.sensor.smooth_window = 0.0
            s._window.clear()
            s.update(0.0, 10, can_close=False)
        self.assertAlmostEqual(narrow[0].estimated_people, 10, delta=0.5)
        self.assertGreater(narrow[0].estimated_wait_s(1.5), 2 * wide[0].estimated_wait_s(1.5))
        self.assertAlmostEqual(narrow[0].estimated_wait_s(1.5), 10 / (1.5 * narrow[0].door_width_m), delta=0.4)


class TestDoorBehaviour(unittest.TestCase):
    def _stress(self, **gate):
        cfg, floor, nav = shared_env()
        cfg.spawn.n_agents = 200
        cfg.sim.seed = 5
        cfg.sensor.count_error = 0.0
        cfg.gate.close_above = 0.05             # absurdly sensitive: forces many closures
        cfg.gate.reopen_at_most = 0.0
        for k, v in gate.items():
            setattr(cfg.gate, k, v)
        return Simulation(cfg, BASE, floor=floor, nav=nav, verbose=False)

    def test_never_closes_the_last_open_exit(self):
        sim = self._stress(min_open_exits=1)
        ever_closed = False
        for _ in range(4000):                    # 40 s
            if sim.done:
                break
            sim.step()
            ever_closed |= any(s.closed for s in sim.signs)
            self.assertTrue(any(s.passable for s in sim.signs), f"all exits closed at t={sim.t:.1f}")
        self.assertTrue(ever_closed, "stress setup never closed a gate; test is meaningless")

    def test_nobody_reroutes_without_noticing_the_closed_gate(self):
        sim = self._stress(notice_distance=0.0)     # nobody is ever close enough to notice
        start = dict(zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist()))
        closed_seen = False
        for _ in range(3000):                       # 30 s
            if sim.done:
                break
            sim.step()
            closed_seen |= any(s.closed for s in sim.signs)
        self.assertTrue(closed_seen)
        for i, tgt in zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist()):
            self.assertEqual(tgt, start[i], f"agent {i} changed exit without noticing anything")

    def test_agents_reroute_once_they_notice(self):
        # block mode: a closed door is a hard fact, everyone who notices it leaves for another exit
        sim = self._stress(notice_distance=15.0, mode="block")
        start = dict(zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist()))
        for _ in range(3000):
            if sim.done:
                break
            sim.step()
        changed = sum(1 for i, t in zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist()) if t != start[i])
        self.assertGreater(changed, 0)

    def test_cost_based_divert_only_switches_when_faster(self):
        # Divert mode: the queue at the other exit is assumed huge -> staying is cheaper, nobody switches.
        sim = self._stress(notice_distance=15.0, mode="divert", cost_based_divert=True)
        for s_ in sim.signs:
            s_.estimated_wait_s = lambda flow, _i=s_.idx: 1000.0 if _i != 0 else 0.0
        start = dict(zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist()))
        sim.signs[0].gate.state = "CLOSED"            # sign 0 says "go elsewhere"
        sim.signs[0].gate.t_changed = sim.t
        for _ in range(1500):
            if sim.done:
                break
            sim._update_awareness()
            sim.t += 0.1
        moved = sum(1 for i, t in zip(sim.crowd.ids.tolist(), sim.crowd.target.tolist())
                    if start[i] == 0 and t != 0)
        self.assertEqual(moved, 0, "people switched although every alternative has a far longer queue")

        # Now the alternatives are empty -> people far from exit 0 do switch.
        sim2 = self._stress(notice_distance=15.0, mode="divert", cost_based_divert=True)
        for s_ in sim2.signs:
            s_.estimated_wait_s = lambda flow, _i=s_.idx: 1000.0 if _i == 0 else 0.0
        sim2.signs[0].gate.state = "CLOSED"
        sim2.signs[0].gate.t_changed = sim2.t
        sim2.crowd.target[:] = 0                      # everybody heads for exit 0
        for _ in range(60):
            sim2.t += 0.5
            sim2._update_awareness()
        self.assertTrue(np.any(sim2.crowd.target != 0), "nobody switched although exit 0 has a 1000 s queue")



if __name__ == "__main__":
    unittest.main()
