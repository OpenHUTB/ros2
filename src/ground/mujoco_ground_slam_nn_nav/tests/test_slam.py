import math
import unittest

import numpy as np

from gvnn.slam.occupancy import OccupancyMap, ScanMatcher


class SlamTest(unittest.TestCase):
    def test_occupancy_update(self):
        grid = OccupancyMap(5.0, 5.0, 0.05)
        ranges = np.full(360, 9.0, dtype=np.float32)
        ranges[0] = 1.0  # forward beam hits a wall 1 m ahead
        grid.update((2.5, 2.5, 0.0), ranges)
        self.assertGreater(float(grid.log_odds[50, 70]), 0.0)   # (3.5, 2.5)
        self.assertLess(float(grid.log_odds[50, 55]), 0.0)      # free space
        self.assertGreater(grid.coverage(), 0.0)

    def test_scan_match_identity(self):
        grid = OccupancyMap(6.0, 6.0, 0.05)
        ranges = np.full(360, 4.0, dtype=np.float32)
        # Build a small square room by synthetic endpoint returns.
        ranges[:] = 2.0
        for _ in range(3):
            grid.update((3.0, 3.0, 0.0), ranges)
        matcher = ScanMatcher(grid, stride=2)
        pose, score = matcher.match((3.0, 3.0, 0.0), ranges)
        self.assertLess(math.hypot(pose[0] - 3.0, pose[1] - 3.0), 0.2)
        self.assertGreaterEqual(score, 0.0)


if __name__ == "__main__":
    unittest.main()
