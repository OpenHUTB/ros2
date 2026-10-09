import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from airsim_rl_planner_ros1.core import observation, velocity
class CoreTests(unittest.TestCase):
    def test_observation(self):
        np.testing.assert_array_equal(observation({'tree'}, [[4.0, 2.0]]), [0,1,0,0,1,0,0,0,0,0])
    def test_vertical_actions(self):
        self.assertLess(velocity(3, 1)[2], 0); self.assertGreater(velocity(4, 1)[2], 0)
    def test_invalid_depth(self):
        for d in ([], [0], [float('nan')], [-1]):
            with self.assertRaises(ValueError): observation(set(), d)
    def test_invalid_action(self):
        for a in (5, -1, 0.5, float('nan')):
            with self.assertRaises(ValueError): velocity(a, 1)
