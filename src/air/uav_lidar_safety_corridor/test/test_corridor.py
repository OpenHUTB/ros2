#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

from corridor import choose_altitude, distance_to_points  # noqa: E402


PARAMS = {
    "lookahead_min": 0.8,
    "lookahead_max": 8.0,
    "half_width": 1.8,
    "vertical_clearance": 0.7,
    "min_offset": -1.0,
    "max_offset": 3.0,
    "offset_step": 0.25,
    "goal_z_weight": 0.08,
    "current_z_weight": 0.04,
    "distance_weight": 0.5,
}


class CorridorTest(unittest.TestCase):
    def test_chooses_altitude_above_obstacle(self):
        points = np.array(
            [
                [1.0, 0.0, 2.0],
                [2.0, 0.0, 2.0],
                [3.0, 0.0, 2.0],
            ],
            dtype=np.float64,
        )
        choice = choose_altitude(
            points, np.array([0.0, 0.0, 2.0]), np.array([5.0, 0.0, 4.0]), PARAMS
        )
        self.assertGreater(choice.target_z, 2.7)
        self.assertEqual(choice.corridor_points, 3)

    def test_empty_cloud_uses_goal_altitude(self):
        choice = choose_altitude(
            np.empty((0, 3)), np.array([0.0, 0.0, 2.0]),
            np.array([5.0, 0.0, 6.0]), PARAMS
        )
        self.assertAlmostEqual(choice.target_z, 6.0)
        self.assertEqual(choice.corridor_points, 0)

    def test_distance_to_empty_cloud_is_infinite(self):
        self.assertTrue(
            np.isinf(distance_to_points(np.zeros(3), np.empty((0, 3))))
        )


if __name__ == "__main__":
    unittest.main()
