"""观测边界、激光逆传感器模型与预设场景可达性。"""
import unittest

import numpy as np

try:
    from scipy.ndimage import binary_dilation
    import torch
    from torch import nn
except ImportError as exc:
    raise unittest.SkipTest("SLAM 测试需要安装 scipy 和 torch 可选依赖") from exc

from mujoco_muscle_gripper.slam_navigation import (
    ANGLES, MAX_RANGE, RESOLUTION, OccupancyMap, ScanSlam, distance_labels, scenarios,
)


class TestSlamNavigation(unittest.TestCase):
    def test_identical_observations_produce_identical_estimates(self):
        calibration = nn.Linear(2, 2, bias=False)
        with torch.no_grad():
            calibration.weight.copy_(torch.eye(2))
        first = ScanSlam(calibration, [1, 1, 0])
        second = ScanSlam(calibration, [1, 1, 0])
        ranges = np.full(len(ANGLES), MAX_RANGE)
        for _ in range(3):
            a = first.observe(np.array([0.2, 0.21]), ranges)
            b = second.observe(np.array([0.2, 0.21]), ranges)
            np.testing.assert_array_equal(a, b)
        self.assertFalse(hasattr(first, "world"))
        self.assertFalse(hasattr(first, "truth"))

    def test_ray_endpoint_free_space_and_unknown_remain_distinct(self):
        grid = OccupancyMap()
        grid.update(np.array([1.1, 1.1, 0]), np.array([1.0]), angles=np.array([0.0]))
        self.assertLess(grid.logodds[5, 7], 0)
        self.assertGreater(grid.logodds[5, 10], 0)
        self.assertEqual(grid.logodds[15, 15], 0)
        empty = OccupancyMap()
        empty.update(np.array([1.1, 1.1, 0]), np.array([MAX_RANGE]), angles=np.array([0.0]))
        self.assertFalse(empty.occupied.any())

    def test_all_layouts_have_a_robot_clearance_route(self):
        for spec in scenarios():
            obstacles = binary_dilation(spec["grid"], iterations=1)
            start = (spec["start"][1::-1] / RESOLUTION).astype(int)
            goal = (spec["goal"][::-1] / RESOLUTION).astype(int)
            self.assertFalse(obstacles[tuple(start)])
            self.assertFalse(obstacles[tuple(goal)])
            self.assertLess(distance_labels(obstacles, tuple(goal))[tuple(start)], 100)


if __name__ == "__main__":
    unittest.main()
