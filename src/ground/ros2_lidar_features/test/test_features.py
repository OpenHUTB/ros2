import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_lidar_features"))

from features import detect_doors, extract_lines, points_from_scan  # noqa: E402
from sim import (Segment, closed_corridor, corridor, evaluate,  # noqa: E402
                 noisy_corridor, raycast)


class TestLines(unittest.TestCase):
    def test_straight_wall(self):
        pts = np.column_stack([np.linspace(0, 4, 100), np.ones(100)])
        lines = extract_lines(pts)
        self.assertEqual(len(lines), 1)
        self.assertLess(lines[0].residual, 0.01)
        self.assertAlmostEqual(lines[0].length, 4.0, delta=0.05)

    def test_scan_filtering_rejects_max_range(self):
        ranges = [1.0, float("inf"), 6.0, 0.0, 1.0]
        pts = points_from_scan(ranges, -0.2, 0.1, max_range=6.0, range_min=0.05)
        self.assertEqual(len(pts), 2)

    def test_empty_and_single_point(self):
        self.assertEqual(len(extract_lines(np.zeros((0, 2)))), 0)
        self.assertEqual(len(extract_lines(np.array([[1.0, 0.0]]))), 0)


class TestDoors(unittest.TestCase):
    def test_door_detected_with_endpoints(self):
        row = evaluate(corridor())
        self.assertEqual(row["doors"], 1)
        self.assertLess(row["width_error_m"], 0.15)
        ranges = raycast(corridor()["segments"], n_beams=720)
        pts = points_from_scan(ranges, -math.pi, 2.0 * math.pi / 720)
        doors = detect_doors(extract_lines(pts))
        self.assertEqual(len(doors), 1)
        xs = sorted([doors[0][0], doors[0][2]])
        self.assertAlmostEqual(xs[0], 3.0, delta=0.15)
        self.assertAlmostEqual(xs[1], 4.0, delta=0.15)

    def test_noisy_scan_still_fits(self):
        row = evaluate(noisy_corridor())
        self.assertEqual(row["doors"], 1)
        self.assertGreater(row["mean_residual_m"], 0.001)
        self.assertLess(row["width_error_m"], 0.15)

    def test_closed_corridor_has_no_door(self):
        row = evaluate(closed_corridor())
        self.assertEqual(row["doors"], 0)
        self.assertEqual(row["lines"], 2)

    def test_narrow_gap_is_not_a_door(self):
        spec = {
            "name": "narrow",
            "segments": [
                Segment(-3.0, 1.2, 3.0, 1.2),
                Segment(3.4, 1.2, 10.0, 1.2),
                Segment(-3.0, -1.2, 10.0, -1.2),
            ],
            "door_width": 0.4,
        }
        self.assertEqual(evaluate(spec)["doors"], 0)

    def test_offset_walls_are_not_a_door(self):
        lines = [
            type("L", (), {"p1": (0.0, 0.0), "p2": (2.0, 0.0),
                           "length": 2.0, "angle": 0.0, "residual": 0.0})(),
            type("L", (), {"p1": (3.0, 0.5), "p2": (5.0, 0.5),
                           "length": 2.0, "angle": 0.0, "residual": 0.0})(),
        ]
        self.assertEqual(len(detect_doors(lines)), 0)


if __name__ == "__main__":
    unittest.main()


