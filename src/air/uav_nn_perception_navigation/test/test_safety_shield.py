#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""safety_shield.py 的单元测试（不依赖 ROS / AirSim）。"""

import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'scripts'))

import safety_shield as ss  # noqa: E402

MAX_RANGE = 40.0


def sectors_with(index, distance):
    """构造 12 维扇区特征，把某个扇区设成指定米制距离。"""
    feats = np.ones(12, dtype=np.float32)
    feats[index] = distance / MAX_RANGE
    return feats


class TestActionSanity(unittest.TestCase):
    def test_nan_action_becomes_hover(self):
        result = ss.apply_shield(float('nan'), 1.0, 0.2, np.ones(12))
        self.assertTrue(result.active)
        self.assertEqual(result.reason, 'non_finite')
        self.assertEqual(result.as_tuple(), (0.0, 0.0, 0.0))

    def test_inf_action_becomes_hover(self):
        result = ss.apply_shield(1.0, float('inf'), 0.2, np.ones(12))
        self.assertTrue(result.active)
        self.assertEqual(result.reason, 'non_finite')

    def test_speed_is_clipped_to_training_range(self):
        result = ss.apply_shield(10.0, 0.0, 0.0, np.ones(12))
        self.assertAlmostEqual(result.vx, ss.MAX_SPEED, places=4)
        self.assertTrue(result.active)
        self.assertEqual(result.reason, 'clip')

    def test_yaw_rate_is_clipped(self):
        result = ss.apply_shield(0.0, 0.0, 9.0, np.ones(12))
        self.assertAlmostEqual(result.yaw_rate, ss.MAX_YAW_RATE, places=4)

    def test_clear_path_passes_through(self):
        result = ss.apply_shield(1.5, -0.5, 0.3, np.ones(12))
        self.assertAlmostEqual(result.vx, 1.5, places=4)
        self.assertAlmostEqual(result.vy, -0.5, places=4)
        self.assertAlmostEqual(result.yaw_rate, 0.3, places=4)
        self.assertFalse(result.active)


class TestObstacleShield(unittest.TestCase):
    def test_front_obstacle_blocks_forward(self):
        # 正前方 0.8 m：禁止前进，但允许后退/平移/转向
        result = ss.apply_shield(2.0, 0.0, 0.2, sectors_with(5, 0.8))
        self.assertEqual(result.vx, 0.0)
        self.assertAlmostEqual(result.yaw_rate, 0.2, places=4)
        self.assertEqual(result.reason, 'front_block')

    def test_front_obstacle_keeps_backward_motion(self):
        result = ss.apply_shield(-1.5, 0.0, 0.0, sectors_with(5, 0.8))
        self.assertAlmostEqual(result.vx, -1.5, places=4)

    def test_very_close_front_obstacle_hovers(self):
        result = ss.apply_shield(2.0, 1.0, 0.5, sectors_with(6, 0.4))
        self.assertEqual(result.reason, 'hard_hover')
        self.assertEqual(result.as_tuple(), (0.0, 0.0, 0.0))

    def test_left_obstacle_blocks_left_translation(self):
        # 机体系 +y 向右，向左平移是 vy < 0；左侧 0.5 m 时禁止继续左移
        result = ss.apply_shield(0.5, -1.0, 0.0, sectors_with(1, 0.5))
        self.assertEqual(result.vy, 0.0)
        self.assertEqual(result.reason, 'left_block')

    def test_right_obstacle_blocks_right_translation(self):
        result = ss.apply_shield(0.5, 1.0, 0.0, sectors_with(10, 0.5))
        self.assertEqual(result.vy, 0.0)
        self.assertEqual(result.reason, 'right_block')

    def test_side_obstacle_does_not_block_other_side(self):
        result = ss.apply_shield(0.5, 1.0, 0.0, sectors_with(1, 0.5))
        self.assertAlmostEqual(result.vy, 1.0, places=4)

    def test_missing_or_short_features_are_safe(self):
        result = ss.apply_shield(1.0, 0.0, 0.0, None)
        self.assertAlmostEqual(result.vx, 1.0, places=4)
        result = ss.apply_shield(1.0, 0.0, 0.0, np.ones(5))
        self.assertAlmostEqual(result.vx, 1.0, places=4)


class TestStaleInput(unittest.TestCase):
    def test_fresh_inputs(self):
        self.assertFalse(ss.is_stale(10.0, 9.9, 9.8))

    def test_stale_lidar(self):
        self.assertTrue(ss.is_stale(10.0, 9.0, 9.9))

    def test_stale_odom(self):
        self.assertTrue(ss.is_stale(10.0, 9.9, 9.0))

    def test_missing_timestamps(self):
        self.assertTrue(ss.is_stale(10.0, None, 9.9))


class TestOscillation(unittest.TestCase):
    def test_small_yaw_is_not_damped(self):
        detector = ss.OscillationDetector()
        out = detector.update(0.1)
        self.assertAlmostEqual(out, 0.1, places=4)

    def test_alternating_yaw_is_damped(self):
        detector = ss.OscillationDetector()
        out = 0.0
        for value in [0.6, -0.6] * 5:
            out = detector.update(value)
        self.assertLessEqual(abs(out), 0.6 * ss.OSC_DAMP + 1e-6)

    def test_monotonic_turn_is_not_damped(self):
        detector = ss.OscillationDetector()
        out = 0.0
        for _ in range(8):
            out = detector.update(0.6)
        self.assertAlmostEqual(out, 0.6, places=4)


if __name__ == '__main__':
    unittest.main()
