import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros2_costmap_inflation"))

from inflation import analytic_distance, inflate  # noqa: E402
from sim import demo_occupancy, run_demo  # noqa: E402


class TestInflation(unittest.TestCase):
    def test_lethal_and_unknown_are_preserved(self):
        occ = np.zeros((20, 20), dtype=np.int8)
        occ[5, 5] = 100
        occ[10, 10] = -1
        result = inflate(occ, resolution=0.1, inflation_radius=0.4)
        self.assertEqual(result.costs[5, 5], 100)
        self.assertEqual(result.costs[10, 10], -1)

    def test_inscribed_band_is_99(self):
        occ = np.zeros((21, 21), dtype=np.int8)
        occ[10, 10] = 100
        result = inflate(occ, resolution=0.1, inflation_radius=0.5,
                         inscribed_radius=0.2, cost_scaling=3.0)
        self.assertEqual(result.costs[10, 11], 99)
        self.assertEqual(result.costs[10, 12], 99)
        self.assertLess(result.costs[10, 13], 99)

    def test_free_beyond_radius(self):
        occ = np.zeros((30, 30), dtype=np.int8)
        occ[15, 15] = 100
        result = inflate(occ, resolution=0.1, inflation_radius=0.3)
        self.assertEqual(result.costs[5, 5], 0)

    def test_cost_decreases_with_distance(self):
        occ = np.zeros((30, 30), dtype=np.int8)
        occ[15, 15] = 100
        result = inflate(occ, resolution=0.1, inflation_radius=0.5)
        self.assertGreater(result.costs[15, 17], result.costs[15, 20])

    def test_exact_distance_error(self):
        occ = demo_occupancy()
        result = inflate(occ, resolution=0.1, inflation_radius=0.6)
        exact = analytic_distance(result.lethal, 0.1)
        err = np.abs(result.distance - exact)[~result.lethal]
        self.assertLess(float(err.max()), 1e-6)

    def test_empty_and_full_maps(self):
        empty = inflate(np.zeros((10, 10), dtype=np.int8), resolution=0.1)
        self.assertTrue((empty.costs == 0).all())
        full = inflate(np.full((10, 10), 100, dtype=np.int8), resolution=0.1)
        self.assertTrue((full.costs == 100).all())

    def test_summary(self):
        metrics = run_demo()
        self.assertGreater(metrics["inflated_cells"], 0)
        self.assertLess(metrics["max_distance_error_m"], 1e-6)


if __name__ == "__main__":
    unittest.main()

