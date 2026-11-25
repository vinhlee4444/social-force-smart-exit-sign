from types import SimpleNamespace
import unittest

import numpy as np

from sfm.config import GuidanceConfig
from sfm.guidance import CongestionGuidance
from sfm.route_advice import RouteCompatible, corridor_segments, opposing_corridor


class OpenFloor:
    def visible_from(self, origin, points):
        return np.ones(len(points), dtype=bool)


class MappedRoute:
    cell = 1.0

    def __init__(self, path, distance):
        self.path = np.array(path)
        self.dist = np.full((12, 12), np.inf)
        self.dist[0, 0] = distance
        x, y = self.path[-1]
        self.dist[y, x] = 0

    def path_prefix(self, start, distance):
        return self.path

    def distance(self, points):
        return np.full(len(points), self.dist[0, 0])

    def direction(self, points):
        return np.tile([1, 0], (len(points), 1))


class RouteCompatibleTests(unittest.TestCase):
    old = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]
    new = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3)]

    def guide(self, other_length=6):
        signs = [SimpleNamespace(camera=np.array([7., .5]), distance_to_door=lambda p: np.full(len(p), 7.))] * 3
        crowd = SimpleNamespace(target=np.array([0]), ids=np.array([0]), vel=np.array([[1., 0.]]),
                                pos=np.array([[.5, .5]]))
        config = GuidanceConfig(policy='route_compatible', switch_probability=1., reaction_delay=(2., 2.))
        sim = SimpleNamespace(floor=OpenFloor(), crowd=crowd, signs=signs, n_total=1,
                              nav=[MappedRoute(self.old, 5), MappedRoute(self.new, other_length),
                                   MappedRoute(self.new, other_length + .25)],
                              cfg=SimpleNamespace(guidance=config, sim=SimpleNamespace(seed=0)))
        guidance = CongestionGuidance(sim)
        guidance.publish(0, ('RED', 'GREEN', 'GREEN'))
        return guidance, crowd

    def test_readable_advice_is_issued_then_rechecked(self):
        guide, crowd = self.guide()
        self.assertEqual(guide.recommendations(0, 0)[0], ())
        guide.update(0)
        self.assertEqual(guide.decisions[-1]['reason'], 'reaction_scheduled')
        self.assertEqual(guide.recommendations(0, 0)[0], (1,))
        guide.update(1.99)
        self.assertEqual(int(crowd.target[0]), 0)
        guide.update(2)
        self.assertEqual(guide.events, [(2, 0, 0, 1)])
        self.assertEqual(int(crowd.target[0]), 1)

    def test_red_alternative_does_not_substitute_another_exit(self):
        guide, crowd = self.guide()
        guide.update(0)
        guide.publish(1, ('RED', 'RED', 'GREEN'))
        self.assertEqual(guide.recommendations(0, 1)[0], ())
        guide.update(2)
        self.assertEqual(guide.decisions[-1]['reason'], 'issued_destination_no_longer_green')
        self.assertEqual(int(crowd.target[0]), 0)

    def test_relative_detour_can_reject_below_absolute_cap(self):
        guide, crowd = self.guide(other_length=8)
        guide.update(0)
        self.assertEqual(guide.decisions[-1]['reason'], 'branch_detour_budget')
        self.assertEqual(guide.recommendations(0, 0)[0], ())
        self.assertEqual(int(crowd.target[0]), 0)

    def test_reverse_direction_on_shared_corridor_rejected_from_static_and_issued_paths(self):
        cfg = GuidanceConfig()
        rules = RouteCompatible(OpenFloor(), cfg)
        old = MappedRoute(self.old, 5)
        # Starts forward, but returns west on a parallel lane in the same corridor.
        back = MappedRoute([(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2)], 6)
        start, heading = np.array([.5, .5]), np.array([1., 0.])
        self.assertEqual(rules.check(old, back, start, heading)[1], 'opposes_mapped_route')
        new = MappedRoute(self.new, 5)
        self.assertEqual(rules.check(old, new, start, -heading)[1], 'no_forward_branch')
        accepted, reason = rules.check(old, new, start, heading)
        self.assertIsNone(reason)
        key, _ = accepted
        opposite = corridor_segments(np.array([(3.5, 3.5), (3.5, 2.5), (3.5, 1.5), (3.5, .5)]))
        rules.issue(1, 0, (key, opposite), 0)
        self.assertEqual(rules.check(old, new, start, heading)[1], 'opposes_issued_route')

    def test_near_parallel_paths_across_wall_do_not_conflict(self):
        class WallFloor:
            def visible_from(self, origin, points):
                return (origin[1] > 1) == (points[:, 1] > 1)

        east = corridor_segments(np.array([(0, .5), (3, .5)]))
        west = corridor_segments(np.array([(3, 1.5), (0, 1.5)]))
        self.assertTrue(opposing_corridor(east, west, OpenFloor(), 2))
        self.assertFalse(opposing_corridor(east, west, WallFloor(), 2))
