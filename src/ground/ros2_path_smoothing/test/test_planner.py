import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_path_smoothing"))

from planner import (Grid, astar, catmull_rom, clearance, curvature,  # noqa: E402
                     path_free, path_length, smooth_path)
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


class TestGeometry(unittest.TestCase):
    def test_to_cell_negative_coordinates(self):
        grid = Grid(np.zeros((10, 10), dtype=bool), resolution=0.1)
        self.assertEqual(grid.to_cell((-0.01, -0.01)), (-1, -1))

    def test_segment_does_not_cut_corner(self):
        occ = np.zeros((10, 10), dtype=bool)
        occ[5, 4] = True
        grid = Grid(occ)
        self.assertFalse(path_free(grid, [(4.05, 5.05), (5.05, 4.05)]))

    def test_catmull_rom_endpoints(self):
        pts = catmull_rom([(0, 0), (1, 1), (2, 0)], samples=8)
        self.assertAlmostEqual(pts[0, 0], 0.0)
        self.assertAlmostEqual(pts[-1, 0], 2.0)


class TestSmoothing(unittest.TestCase):
    def test_l_path_curvature_is_reduced(self):
        occ = np.zeros((60, 60), dtype=bool)
        grid = Grid(occ)
        path = [(5, y) for y in range(5, 50)] + \
               [(x, 49) for x in range(6, 55)]
        raw = np.asarray([grid.to_world(c) for c in path])
        smooth, safe = smooth_path(grid, path)
        self.assertTrue(safe)
        self.assertLess(float(curvature(smooth).max()),
                        float(curvature(raw).max()))

    def test_two_point_path_is_checked(self):
        occ = np.zeros((30, 30), dtype=bool)
        occ[15, 15] = True
        grid = Grid(occ)
        path = [(2, 15), (27, 15)]
        smooth, safe = smooth_path(grid, path)
        self.assertFalse(safe)
        self.assertFalse(path_free(grid, smooth))

    def test_scenarios_are_safe_independently(self):
        for factory in (corridor, doorway, clutter):
            spec = factory()
            row = evaluate(spec)
            cells = astar(spec.grid, spec.start, spec.goal)
            smooth, safe = smooth_path(spec.grid, cells)
            self.assertTrue(row["ok"])
            self.assertTrue(path_free(spec.grid, smooth))
            self.assertGreaterEqual(clearance(spec.grid, smooth), 0.1)
            self.assertTrue(safe)
            self.assertLessEqual(row["smooth_max_curvature"],
                                 row["raw_max_curvature"])


if __name__ == "__main__":
    unittest.main()

