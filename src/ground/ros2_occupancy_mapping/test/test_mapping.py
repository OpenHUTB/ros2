import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_occupancy_mapping"))

from mapping import OCCUPIED, UNKNOWN, LogOddsGrid, bresenham  # noqa: E402
from sim import run_mapping  # noqa: E402


class TestBresenham(unittest.TestCase):
    def test_horizontal_line(self):
        self.assertEqual(bresenham(0, 0, 3, 0), [(0, 0), (1, 0), (2, 0), (3, 0)])

    def test_diagonal_line(self):
        self.assertEqual(bresenham(0, 0, 2, 2), [(0, 0), (1, 1), (2, 2)])


class TestGrid(unittest.TestCase):
    def test_unknown_before_update(self):
        grid = LogOddsGrid(-1.0, -1.0, 1.0, 1.0, resolution=0.1)
        self.assertTrue((grid.as_occupancy() == UNKNOWN).all())

    def test_ray_marks_free_and_occupied(self):
        grid = LogOddsGrid(-1.0, -1.0, 4.0, 1.0, resolution=0.1)
        grid.update_scan((0.0, 0.0, 0.0), [3.0], 0.0, 0.0, max_range=8.0)
        occ = grid.as_occupancy()
        free_gx, free_gy = grid.world_to_grid(1.0, 0.0)
        hit_gx, hit_gy = grid.world_to_grid(3.0, 0.0)
        self.assertEqual(occ[free_gy, free_gx], 0)
        self.assertEqual(occ[hit_gy, hit_gx], OCCUPIED)


class TestMappingQuality(unittest.TestCase):
    def test_map_quality(self):
        _, metrics = run_mapping()
        self.assertGreater(metrics["coverage"], 0.5)
        self.assertGreaterEqual(metrics["occupied_iou"], 0.6)
        self.assertGreaterEqual(metrics["occupied_precision"], 0.8)


if __name__ == "__main__":
    unittest.main()
