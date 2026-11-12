#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""safety_shield 的单元测试（纯 NumPy，不依赖 ROS/AirSim）。"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'scripts'))

from safety_shield import (  # noqa: E402
    FORWARD, FORWARD_LEFT, FORWARD_RIGHT, HOVER, TURN_LEFT, TURN_RIGHT,
    action_mask, select_action, front_distance_m, recovery_turn,
)


def sectors_with(front=None, left=None, right=None):
    """构造 24 扇区：距离（米）转成归一化值，None 表示通畅。"""
    d = np.full(24, 40.0, dtype=np.float32)
    if front is not None:
        d[8:16] = front
    if left is not None:
        d[0:8] = left
    if right is not None:
        d[16:24] = right
    return d / 40.0


class TestActionMask(unittest.TestCase):
    def test_clear_path_allows_everything(self):
        self.assertTrue(action_mask(sectors_with()).all())

    def test_front_wall_blocks_all_forward_actions(self):
        mask = action_mask(sectors_with(front=0.4))
        self.assertFalse(mask[FORWARD])
        self.assertFalse(mask[FORWARD_LEFT])
        self.assertFalse(mask[FORWARD_RIGHT])
        self.assertTrue(mask[HOVER])
        self.assertTrue(mask[TURN_LEFT])
        self.assertTrue(mask[TURN_RIGHT])

    def test_side_wall_only_blocks_that_side(self):
        mask = action_mask(sectors_with(right=0.3))
        self.assertTrue(mask[FORWARD])
        self.assertTrue(mask[FORWARD_LEFT])
        self.assertFalse(mask[FORWARD_RIGHT])

    def test_invalid_sector_count_is_permissive(self):
        self.assertTrue(action_mask(np.ones(5, dtype=np.float32)).all())


class TestSelectAction(unittest.TestCase):
    def test_side_wall_does_not_trigger_front_stop(self):
        sectors = sectors_with(left=0.3)
        self.assertAlmostEqual(front_distance_m(sectors), 40.0)

    def test_near_front_wall_turns_toward_open_side(self):
        sectors = sectors_with(front=0.4, left=0.3, right=2.0)
        self.assertEqual(recovery_turn(sectors), TURN_RIGHT)

    def test_forward_kept_when_clear(self):
        q = np.array([0.0, 10.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)
        self.assertEqual(select_action(q, sectors_with()), FORWARD)

    def test_forward_masked_when_wall_ahead(self):
        # 网络仍偏好 forward，但安全层必须把前向动作屏蔽掉
        q = np.array([0.0, 10.0, 9.0, 8.0, 2.5, 2.0], dtype=np.float32)
        action = select_action(q, sectors_with(front=0.4))
        self.assertIn(action, (HOVER, TURN_LEFT, TURN_RIGHT))
        self.assertEqual(action, TURN_LEFT)

    def test_forward_right_masked_when_right_wall(self):
        q = np.array([0.0, 1.0, 2.0, 10.0, 1.0, 1.0], dtype=np.float32)
        action = select_action(q, sectors_with(right=0.3))
        self.assertNotEqual(action, FORWARD_RIGHT)
        self.assertEqual(action, FORWARD_LEFT)


if __name__ == '__main__':
    unittest.main(verbosity=2)
