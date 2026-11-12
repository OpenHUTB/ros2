#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制器核心离线单元测试，不依赖 rclpy / ROS 2。

运行：
    python3 -m unittest discover -s tests -v
或：
    pytest tests/test_centering.py
"""

import math
import unittest

from diff_drive_corridor_centering.centering import (
    CenteringParams,
    CorridorCenteringController,
    diff_drive_step,
)


class TestCenteringController(unittest.TestCase):
    def setUp(self) -> None:
        self.ctrl = CorridorCenteringController(CenteringParams())

    def test_centered_inputs_keep_straight(self) -> None:
        # 左右墙等距、前方通畅：偏差为 0，角速度接近 0，车速接近上限。
        r = self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=6.0, now=0.0)
        self.assertAlmostEqual(r["e"], 0.0, places=6)
        self.assertAlmostEqual(r["w"], 0.0, places=6)
        self.assertAlmostEqual(r["v"], self.ctrl.params.v_max, places=6)
        self.assertEqual(r["state"], "CENTER")

    def test_drift_right_corrects_left(self) -> None:
        # 车体偏右：右墙更近（d_right 小），e > 0，应输出正向（向左）角速度。
        r = self.ctrl.step(d_left=1.3, d_right=0.7, d_forward=6.0, now=0.0)
        self.assertGreater(r["e"], 0.0)
        self.assertGreater(r["w"], 0.0)

    def test_drift_left_corrects_right(self) -> None:
        # 车体偏左：左墙更近（d_left 小），e < 0，应输出负向（向右）角速度。
        r = self.ctrl.step(d_left=0.7, d_right=1.3, d_forward=6.0, now=0.0)
        self.assertLess(r["e"], 0.0)
        self.assertLess(r["w"], 0.0)

    def test_angular_velocity_clipped(self) -> None:
        # 大偏差下角速度仍被限制在 w_max 内。
        r = self.ctrl.step(d_left=2.0, d_right=0.1, d_forward=6.0, now=0.0)
        self.assertLessEqual(abs(r["w"]), self.ctrl.params.w_max + 1e-9)

    def test_blocked_forward_stops_and_recovers(self) -> None:
        # 前方过近进入 BLOCKED 并零速；距离恢复到阈值后回到 CENTER。
        blocked = self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=0.3, now=0.0)
        self.assertEqual(blocked["state"], "BLOCKED")
        self.assertEqual(blocked["v"], 0.0)
        self.assertEqual(blocked["w"], 0.0)

        recovered = self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=1.2, now=0.1)
        self.assertEqual(recovered["state"], "CENTER")
        self.assertGreater(recovered["v"], 0.0)

    def test_scan_timeout_recovers_after_stream_resumes(self) -> None:
        # 测距超时进入 BLOCKED 后，恢复收到测距应能回到巡航，不会被永久锁死。
        self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=6.0, now=0.0)
        # 断流超过 scan_timeout，进入 BLOCKED。
        blocked = self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=6.0, now=1.0)
        self.assertEqual(blocked["state"], "BLOCKED")
        self.assertEqual(blocked["v"], 0.0)
        # 恢复测距后下一帧应退出 BLOCKED 继续巡航。
        recovered = self.ctrl.step(d_left=1.0, d_right=1.0, d_forward=6.0, now=1.1)
        self.assertEqual(recovered["state"], "CENTER")
        self.assertGreater(recovered["v"], 0.0)

    def test_derivative_zero_on_first_sample(self) -> None:
        r = self.ctrl.step(d_left=1.2, d_right=0.8, d_forward=6.0, now=0.0)
        self.assertAlmostEqual(r["de"], 0.0, places=6)

    def test_speed_decays_with_turn_rate(self) -> None:
        # 转角越大，前进速度越低（不低于 v_floor）；各自使用全新控制器，
        # 避免上一步的微分历史影响本轮输出。
        straight = CorridorCenteringController(CenteringParams()).step(
            1.0, 1.0, 6.0, 0.0)
        turning = CorridorCenteringController(CenteringParams()).step(
            1.5, 0.5, 6.0, 0.0)
        self.assertLess(turning["v"], straight["v"])
        self.assertGreaterEqual(turning["v"], self.ctrl.params.v_floor - 1e-9)

    def test_diff_drive_kinematics(self) -> None:
        pose = (0.0, 0.0, 0.0)
        nxt = diff_drive_step(pose, v=1.0, w=0.0, dt=0.1)
        self.assertAlmostEqual(nxt[0], 0.1, places=6)
        self.assertAlmostEqual(nxt[1], 0.0, places=6)
        self.assertAlmostEqual(nxt[2], 0.0, places=6)

        nxt = diff_drive_step(pose, v=0.0, w=1.0, dt=0.1)
        self.assertAlmostEqual(nxt[2], 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
