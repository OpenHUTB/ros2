import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_path_smoothing"))

from planner import (Grid, astar, catmull_rom, curvature, path_length,  # noqa: E402
                     smooth_path)
from sim import corridor, doorway, clutter, evaluate  # noqa: E402


class TestAStar(unittest.TestCase):
    def test_astar_returns_free_path(self):
        occ = np.zeros((50, 50), dtype=bool)
        occ[:, 25] = True
        occ[20:30, 25] = False
        grid = Grid(occ)
        path = astar(grid, (2, 5), (47, 45))
        self.assertTrue(path)
        self.assertTrue(all(grid.free(c) for c in path))


class TestCatmullRom(unittest.TestCase):
    def test_endpoints_are_preserved(self):
        pts = catmull_rom([(0, 0), (1, 1), (2, 0)], samples=8)
        self.assertAlmostEqual(pts[0, 0], 0.0)
        self.assertAlmostEqual(pts[0, 1], 0.0)
        self.assertAlmostEqual(pts[-1, 0], 2.0)
        self.assertAlmostEqual(pts[-1, 1], 0.0)

    def test_circle_curvature(self):
        pts = [(math.cos(t), math.sin(t)) for t in
               [i * 2 * math.pi / 40 for i in range(41)]]
        k = curvature(pts)
        self.assertAlmostEqual(float(k.mean()), 1.0, delta=0.05)


class TestSmoothing(unittest.TestCase):
    def test_l_path_curvature_is_reduced(self):
        occ = np.zeros((60, 60), dtype=bool)
        grid = Grid(occ)
        path = [(5, 5)] + [(5, y) for y in range(5, 50)] + \
               [(x, 49) for x in range(5, 55)]
        raw = np.asarray([grid.to_world(c) for c in path])
        smooth, safe = smooth_path(grid, path)
        self.assertTrue(safe)
        self.assertLess(abs(path_length(smooth) - path_length(raw)), 4.0)

    def test_scenarios_are_safe(self):
        for factory in (corridor, doorway, clutter):
            row = evaluate(factory())
            self.assertTrue(row["ok"])
            self.assertTrue(row["smooth_safe"])
            self.assertLessEqual(row["smooth_max_curvature"],
                                 row["raw_max_curvature"])


if __name__ == "__main__":
    unittest.main()



