"""规划器不变量测试。

无需 ROS 环境即可运行：

    python3 -m unittest discover -s tests -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from parking_path_planner.planner import (  # noqa: E402
    PlannerConfig,
    SlotParams,
    VehicleParams,
    count_gear_shifts,
    densify,
    goal_pose_from_slot,
    is_collision,
    max_curvature,
    plan_parking,
    wrap_angle,
)


class PlannerInvariantTest(unittest.TestCase):
    """固定场景下的规划结果应满足全部约束。"""

    def setUp(self):
        self.vehicle = VehicleParams()
        self.slot = SlotParams()
        self.start = (-7.0, -4.20, 0.0)
        self.goal = goal_pose_from_slot(self.slot, self.vehicle)
        self.result = plan_parking(self.start, self.goal, self.vehicle,
                                   self.slot, PlannerConfig())

    def test_plan_succeeds(self):
        self.assertTrue(self.result.success, self.result.message)
        self.assertGreater(len(self.result.poses), 5)

    def test_goal_within_tolerance(self):
        config = PlannerConfig()
        last = self.result.poses[-1]
        distance = math.hypot(last[0] - self.goal[0], last[1] - self.goal[1])
        self.assertLess(distance, config.goal_xy_tolerance + 1e-6)
        self.assertLess(abs(wrap_angle(last[2] - self.goal[2])),
                        math.radians(config.goal_heading_tolerance_deg) + 1e-6)

    def test_curvature_respects_min_turn_radius(self):
        """轨迹曲率不得超过车辆最小转弯半径对应的上限。"""
        self.assertLessEqual(max_curvature(self.result.poses),
                             1.0 / self.vehicle.min_turn_radius + 1e-3)

    def test_no_collision_along_path(self):
        for x, y, yaw, _gear in self.result.poses:
            self.assertFalse(is_collision(x, y, yaw, self.vehicle, self.slot))

    def test_gear_shifts_are_limited(self):
        self.assertLessEqual(count_gear_shifts(self.result.poses), 3)

    def test_reverse_segment_exists(self):
        """倒车入库，轨迹中必须存在倒车段。"""
        self.assertTrue(any(pose[3] < 0 for pose in self.result.poses))

    def test_densify_keeps_endpoints(self):
        dense = densify(self.result.poses)
        self.assertGreater(len(dense), len(self.result.poses))
        self.assertAlmostEqual(dense[0][0], self.result.poses[0][0], places=6)
        self.assertAlmostEqual(dense[-1][1], self.result.poses[-1][1], places=6)

    def test_neighbors_not_parked_still_works(self):
        config = PlannerConfig(neighbors_parked=False)
        result = plan_parking(self.start, self.goal, self.vehicle, self.slot, config)
        self.assertTrue(result.success, result.message)


if __name__ == '__main__':
    unittest.main()
