# -*- coding: utf-8 -*-
"""Standard-library tests for the lidar avoidance control logic.

No rclpy / Gazebo / ROS workspace is required:

    python -m unittest discover -s tests -v
    python -m pytest tests -q
"""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(HERE)
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from tb3_lidar_obstacle import avoid_logic as logic  # noqa: E402


def scan(n=360, angle_min=-math.pi, angle_max=math.pi, value=10.0):
    inc = (angle_max - angle_min) / n
    return [value] * n, angle_min, inc


class SectorTest(unittest.TestCase):
    def test_angle_min_nonzero_uses_correct_front_index(self):
        # 0 rad (front) is at index 180, not index 0.
        ranges, amin, inc = scan(n=360, angle_min=-math.pi, value=10.0)
        ranges[180] = 0.3
        d = logic.sector_min(ranges, amin, inc, 0.0, 15.0)
        self.assertAlmostEqual(d, 0.3, places=6)

    def test_ranges_zero_index_is_not_blindly_used(self):
        ranges, amin, inc = scan(n=360, angle_min=-math.pi, value=10.0)
        ranges[0] = 0.1  # rear / scanning start, must not affect front
        d = logic.sector_min(ranges, amin, inc, 0.0, 15.0)
        self.assertAlmostEqual(d, 10.0, places=6)

    def test_filters_inf_nan_and_out_of_range(self):
        ranges, amin, inc = scan(n=360, angle_min=-math.pi, value=float('inf'))
        ranges[180] = float('nan')
        ranges[181] = 0.0            # below range_min
        ranges[182] = 99.0           # above range_max
        ranges[183] = 0.4            # valid
        d = logic.sector_min(ranges, amin, inc, 0.0, 15.0,
                             range_min=0.12, range_max=3.5)
        self.assertAlmostEqual(d, 0.4, places=6)

    def test_empty_sector_is_infinity(self):
        ranges, amin, inc = scan(n=360, value=float('inf'))
        self.assertTrue(math.isinf(
            logic.sector_min(ranges, amin, inc, 0.0, 15.0)))


class DecisionTest(unittest.TestCase):
    def test_clear_ahead_goes_forward(self):
        self.assertEqual(logic.decide(2.0),
                         (logic.LINEAR_SPEED, 0.0, 'forward'))

    def test_obstacle_without_side_info_turns_left(self):
        self.assertEqual(logic.decide(0.2),
                         (0.0, logic.TURN_SPEED, 'turn_left'))

    def test_chooses_clearer_side(self):
        self.assertEqual(logic.decide(0.2, left=3.0, right=0.4),
                         (0.0, logic.TURN_SPEED, 'turn_left'))
        self.assertEqual(logic.decide(0.2, left=0.4, right=3.0),
                         (0.0, -logic.TURN_SPEED, 'turn_right'))

    def test_equal_sides_default_left(self):
        self.assertEqual(logic.decide(0.2, left=1.0, right=1.0),
                         (0.0, logic.TURN_SPEED, 'turn_left'))

    def test_end_to_end_scan_to_decision(self):
        ranges, amin, inc = scan(n=360, angle_min=-math.pi, value=10.0)
        # ROS LaserScan: positive angle = left, negative angle = right.
        # front blocked; right sector blocked; left sector stays clear.
        for i in range(len(ranges)):
            angle = amin + i * inc
            if abs(angle) < math.radians(15):
                ranges[i] = 0.3
            elif -math.radians(45) <= angle <= -math.radians(15):
                ranges[i] = 0.25
        linear_x, angular_z, state = logic.scan_to_decision(
            ranges, amin, inc)
        self.assertEqual(state, 'turn_left')  # clearer side is left (+angle)
        self.assertEqual(linear_x, 0.0)
        self.assertAlmostEqual(angular_z, logic.TURN_SPEED, places=6)


class WatchdogContractTest(unittest.TestCase):
    def test_timeout_and_stop_command_exist(self):
        self.assertGreater(logic.SCAN_TIMEOUT_S, 0.0)
        self.assertEqual(logic.STOP[:2], (0.0, 0.0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
