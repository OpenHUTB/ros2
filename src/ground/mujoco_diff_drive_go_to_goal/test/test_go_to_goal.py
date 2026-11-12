#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""go-to-goal 纯逻辑单元测试（不加载 MuJoCo 模型，不需要图形界面）。

用法（在包目录下执行）::

    python3 test/test_go_to_goal.py
    python3 -m unittest discover -s test -p "test_*.py"

用 ``unittest.TestCase`` 组织，断言走 ``self.assert*`` 方法而不是裸 ``assert``
语句：

- 可被 ``unittest`` / pytest / nose（``catkin_add_nosetests``）自动收集；
- 断言是方法调用，``python3 -O`` 优化模式下不会被编译期剔除，仍会真正校验。

覆盖：角度归一化、四元数解 yaw、差速逆解、到达判据、转向/直行模式与滞回。
"""
import os
import sys
import math
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import main as m  # noqa: E402


class TestGoToGoal(unittest.TestCase):

    def test_wrap_angle(self):
        self.assertAlmostEqual(m.wrap_angle(0.0), 0.0, places=12)
        self.assertAlmostEqual(m.wrap_angle(math.pi), math.pi, places=12)
        # 超过 pi 折回 (-pi, pi]
        self.assertAlmostEqual(m.wrap_angle(3 * math.pi / 2), -math.pi / 2, places=12)
        self.assertAlmostEqual(m.wrap_angle(-3 * math.pi / 2), math.pi / 2, places=12)

    def test_quat_to_yaw(self):
        # 绕 z 轴转 90°：四元数 [qw,qx,qy,qz] = [cos45, 0, 0, sin45]
        q = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
        yaw = m.quat_to_yaw(q)
        self.assertAlmostEqual(yaw, math.pi / 2, places=12, msg=f"yaw={yaw}")
        # 回归原 bug：qpos[5] 是 qy，绕 z 转时恒为 0，不能当 yaw
        self.assertAlmostEqual(q[2], 0.0, places=12)
        # 单位四元数 → yaw 0
        self.assertAlmostEqual(m.quat_to_yaw([1.0, 0.0, 0.0, 0.0]), 0.0, places=12)

    def test_diff_drive_vels(self):
        # 直行 omega=0：左右轮等速
        wl, wr = m.diff_drive_vels(0.6, 0.0)
        self.assertAlmostEqual(wl, wr, places=12)
        self.assertAlmostEqual(wl, 0.6 / m.WHEEL_RADIUS, places=12)
        # 原地转 v=0：左右轮等大反号
        wl, wr = m.diff_drive_vels(0.0, 1.0)
        self.assertAlmostEqual(wl + wr, 0.0, places=12)
        self.assertAlmostEqual(wr - wl, 1.0 * m.WHEEL_BASE / m.WHEEL_RADIUS, places=12)

    def test_arrive(self):
        # 就在目标点 → 到达、两轮速度 0
        left, right, _, arrived, _, dist = m.control_step(
            m.GOAL_X, m.GOAL_Y, 0.0, "drive")
        self.assertTrue(arrived)
        self.assertEqual(left, 0.0)
        self.assertEqual(right, 0.0)
        self.assertLess(dist, m.ARRIVE_DIST)

    def test_turn_then_drive_hysteresis(self):
        # 初始车头朝 +x，目标在左前方，初始角度差 > ANGLE_HI → 原地转向
        left, right, mode, arrived, ad, _ = m.control_step(0.0, 0.0, 0.0, "drive")
        self.assertFalse(arrived)
        self.assertEqual(mode, "turn")
        self.assertGreater(ad, m.ANGLE_HI)
        # 转向轮速：左轮反转、右轮正转（车头向左）
        self.assertLess(left, 0.0)
        self.assertGreater(right, 0.0)

        # 滞回：已在转向，角度差降到 ANGLE_LO 以下才回直行
        left, right, mode, arrived, _, _ = m.control_step(0.1, 0.05, 0.3, "turn")
        self.assertEqual(mode, "drive")
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)


if __name__ == "__main__":
    unittest.main()
