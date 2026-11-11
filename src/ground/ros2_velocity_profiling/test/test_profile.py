import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_velocity_profiling"))

from profile import curvature, speed_profile  # noqa: E402
from sim import circle_points, evaluate, slalom_points  # noqa: E402


class TestCurvature(unittest.TestCase):
    def test_circle_curvature(self):
        pts = circle_points(radius=1.2, count=400)
        k = curvature(pts)
        self.assertAlmostEqual(float(k.mean()), 1.0 / 1.2, delta=0.02)


class TestProfile(unittest.TestCase):
    def test_limits_are_respected(self):
        prof = speed_profile(slalom_points(), v_max=1.0, a_max=0.8, a_lat_max=0.5)
        self.assertLessEqual(prof.max_lateral_accel, 0.5 + 1e-3)
        self.assertLessEqual(prof.max_accel, 0.8 + 1e-2)
        self.assertLessEqual(float(prof.speed.max()), 1.0 + 1e-9)
        self.assertGreaterEqual(float(prof.speed.min()), 0.12 - 1e-9)

    def test_profile_is_slower_than_constant(self):
        prof = speed_profile(circle_points(), v_max=1.0, a_max=0.8, a_lat_max=0.5)
        self.assertGreater(prof.duration, 2 * math.pi * 1.2 / 1.0)

    def test_metrics_table(self):
        row = evaluate("circle", circle_points())
        self.assertLessEqual(row["profile_max_lat"], 0.501)
        self.assertGreater(row["profile_time_s"], row["constant_time_s"])


if __name__ == "__main__":
    unittest.main()
