import unittest

import numpy as np

from sfm.route_advice import branch_window


class ForwardBranchTests(unittest.TestCase):
    def test_approve_ahead_and_reject_after_branch_or_turnback(self):
        old = np.array([[0, 0], [1, 0], [2, 0], [3, 0]])
        branch = np.array([[0, 0], [1, 0], [1, 1], [1, 2]])
        self.assertTrue(branch_window(old, branch, np.array([1, 0]), 1, 8))
        self.assertFalse(branch_window(old[1:], branch[1:], np.array([1, 0]), 1, 8))
        self.assertFalse(branch_window(old, branch, np.array([-1, 0]), 1, 8))
        backtrack = np.array([[0, 0], [1, 0], [0, 1], [-1, 1]])
        self.assertFalse(branch_window(old, backtrack, np.array([1, 0]), 1, 8))

    def test_same_route_is_not_a_branch(self):
        path = np.array([[0, 0], [1, 0], [2, 0]])
        self.assertFalse(branch_window(path, path, np.array([1, 0]), 1, 8))
