"""core.py 的离线单元测试。"""

import math
import unittest

from pure_pursuit_reverse import core

PATH = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]


class TestAngles(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(core.wrap_angle(3 * math.pi), math.pi, places=6)
        self.assertAlmostEqual(core.wrap_angle(-3 * math.pi), math.pi, places=6)
        self.assertAlmostEqual(core.wrap_angle(0.3), 0.3)


class TestProjection(unittest.TestCase):
    def test_distance_on_straight(self):
        s, lateral, point = core.project_to_path((1.0, 0.5, 0.0), PATH)
        self.assertAlmostEqual(point[0], 1.0, places=6)
        self.assertAlmostEqual(point[1], 0.0, places=6)
        self.assertAlmostEqual(abs(lateral), 0.5, places=6)
        self.assertAlmostEqual(s, 1.0, places=6)

    def test_path_length(self):
        self.assertAlmostEqual(core.path_length(PATH), 4.0)


class TestLookahead(unittest.TestCase):
    def test_advances_along_path(self):
        s, _, _ = core.project_to_path((0.5, 0.0, 0.0), PATH)
        target = core.lookahead_point(PATH, s, 1.5)
        self.assertAlmostEqual(target[0], 2.0, places=6)

    def test_clamps_at_path_end(self):
        target = core.lookahead_point(PATH, 3.5, 5.0)
        self.assertAlmostEqual(target[0], 4.0, places=6)


class TestCurvature(unittest.TestCase):
    def test_straight_ahead_is_zero(self):
        kappa, dist = core.pure_pursuit_curvature((0.0, 0.0, 0.0), (2.0, 0.0))
        self.assertAlmostEqual(kappa, 0.0, places=6)
        self.assertAlmostEqual(dist, 2.0)

    def test_straight_behind_for_reverse_is_zero(self):
        kappa, _ = core.pure_pursuit_curvature((2.0, 0.0, 0.0), (0.0, 0.0), reverse=True)
        self.assertAlmostEqual(kappa, 0.0, places=6)

    def test_offset_target_gives_curvature(self):
        kappa, _ = core.pure_pursuit_curvature((0.0, 0.0, 0.0), (1.0, 1.0))
        self.assertGreater(kappa, 0.0)

    def test_short_target_is_ignored(self):
        kappa, dist = core.pure_pursuit_curvature((0.0, 0.0, 0.0), (0.01, 0.0), min_lookahead=0.05)
        self.assertEqual(kappa, 0.0)
        self.assertLess(dist, 0.05)


class TestRateLimit(unittest.TestCase):
    def test_clip(self):
        self.assertAlmostEqual(core.rate_limit(0.0, 1.0, 0.2), 0.2)
        self.assertAlmostEqual(core.rate_limit(0.0, -1.0, 0.2), -0.2)
        self.assertAlmostEqual(core.rate_limit(0.0, 0.1, 0.2), 0.1)


if __name__ == "__main__":
    unittest.main()
