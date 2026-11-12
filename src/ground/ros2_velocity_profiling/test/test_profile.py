import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_velocity_profiling"))

from profile import curvature, speed_profile  # noqa: E402
from sim import circle_points, evaluate, slalom_points, square_points  # noqa: E402


class TestCurvature(unittest.TestCase):
    def test_circle_curvature_signed(self):
        pts = circle_points(radius=1.2, count=400)
        k = curvature(pts)
        self.assertAlmostEqual(float(k.mean()), 1.0 / 1.2, delta=0.02)

    def test_straight_line_is_zero(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        self.assertAlmostEqual(float(np.max(np.abs(curvature(pts)))), 0.0)

    def test_right_turn_is_negative(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (1.0, -1.0)]
        self.assertLess(float(curvature(pts)[1]), 0.0)

    def test_reversal_is_large(self):
        pts = [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]
        self.assertGreater(float(curvature(pts)[1]), 1.0)


class TestProfile(unittest.TestCase):
    def test_endpoints_stop(self):
        prof = speed_profile(slalom_points(), v_start=0.0, v_end=0.0)
        self.assertAlmostEqual(float(prof.speed[0]), 0.0)
        self.assertAlmostEqual(float(prof.speed[-1]), 0.0)

    def test_limits_are_respected(self):
        prof = speed_profile(slalom_points(), v_max=1.0, a_max=0.8, a_lat_max=0.5)
        self.assertLessEqual(prof.max_lateral_accel, 0.5 + 1e-3)
        self.assertLessEqual(prof.max_accel, 0.8 + 1e-2)
        self.assertLessEqual(float(prof.speed.max()), 1.0 + 1e-9)

    def test_w_sign_follows_curvature(self):
        prof = speed_profile([(0.0, 0.0), (1.0, 0.0), (1.0, -1.0)])
        self.assertLess(float(prof.curvature[1]), 0.0)
        self.assertLess(float(prof.curvature[1] * prof.speed[1]), 0.0)

    def test_duplicate_points_are_tolerated(self):
        prof = speed_profile([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
        self.assertEqual(len(prof.speed), 4)
        self.assertTrue(np.isfinite(prof.curvature).all())

    def test_time_increase_is_bounded(self):
        for factory in (circle_points, slalom_points, square_points):
            row = evaluate("path", factory())
            ratio = row["profile_time_s"] / row["constant_time_s"]
            self.assertGreater(ratio, 1.1)
            self.assertLess(ratio, 2.0)

    def test_two_points_are_valid(self):
        prof = speed_profile([(0.0, 0.0), (2.0, 0.0)])
        self.assertEqual(len(prof.speed), 2)
        self.assertGreater(prof.duration, 0.0)

    def test_invalid_parameters_raise(self):
        with self.assertRaises(ValueError):
            speed_profile(circle_points(), v_max=0.0)
        with self.assertRaises(ValueError):
            speed_profile(circle_points(), min_speed=2.0)
        with self.assertRaises(ValueError):
            speed_profile([(0.0, 0.0)])

    def test_metrics_table(self):
        for factory in (circle_points, slalom_points, square_points):
            row = evaluate("path", factory())
            self.assertLessEqual(row["profile_max_lat"], 0.501)
            self.assertGreater(row["profile_time_s"], row["constant_time_s"])


if __name__ == "__main__":
    unittest.main()

