# -*- coding: utf-8 -*-
"""补偿控制器、风险门控与轮速差门限测试（不依赖 MuJoCo）。"""
import unittest

from mujoco_car_drive_fault.config import Config
from mujoco_car_drive_fault.guard import FaultGuard, comp_steer, rule_gate


class TestCompSteer(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_no_error_no_correction(self):
        s = comp_steer(0.0, 0.0, 0.0, self.cfg)
        self.assertAlmostEqual(s, 0.0)

    def test_left_drift_steers_right(self):
        # 横向偏差为正（车往左偏），修正应为负（往右打）
        s = comp_steer(0.3, 0.0, 0.0, self.cfg)
        self.assertLess(s, 0.0)

    def test_right_drift_steers_left(self):
        s = comp_steer(-0.3, 0.0, 0.0, self.cfg)
        self.assertGreater(s, 0.0)

    def test_positive_yaw_rate_steers_right(self):
        s = comp_steer(0.0, 0.0, 0.4, self.cfg)
        self.assertLess(s, 0.0)

    def test_correction_is_limited(self):
        s = comp_steer(5.0, 3.0, 4.0, self.cfg)
        self.assertLessEqual(abs(s), self.cfg.comp_limit + 1e-9)

    def test_zero_gain_returns_zero(self):
        s = comp_steer(0.5, 0.5, 0.5, self.cfg, gain=0.0)
        self.assertAlmostEqual(s, 0.0)


class TestFaultGuard(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_none_mode_never_compensates(self):
        g = FaultGuard(self.cfg)
        for _ in range(3):
            gain, action = g.decide(0.99, 0.8, 30.0, mode="none")
        self.assertEqual(gain, 0.0)
        self.assertEqual(action, "pass")

    def test_rule_needs_pose_error(self):
        g = FaultGuard(self.cfg)
        gain, action = g.decide(0.0, 0.01, 0.2, mode="rule")
        self.assertEqual(gain, 0.0)
        self.assertEqual(action, "pass")

    def test_rule_engages_above_threshold(self):
        g = FaultGuard(self.cfg)
        gain, action = g.decide(0.0, self.cfg.rule_y + 0.2, 0.2, mode="rule")
        self.assertGreater(gain, 0.0)
        self.assertEqual(action, "assist")

    def test_nn_engages_below_speed_difference(self):
        # 轮速差还很小的时候网络已经给出风险，这组对比是本题的核心差别。
        # 门控对风险做 EMA 平滑，持续几帧同一风险才会被采信，所以这里连续喂帧。
        g = FaultGuard(self.cfg)
        gain, action = 0.0, "pass"
        for _ in range(6):
            gain, action = g.decide(0.7, 0.01, 0.2, mode="nn")
        self.assertGreater(gain, 0.0)
        self.assertEqual(action, "assist")

    def test_nn_high_risk_requests_slow_down(self):
        g = FaultGuard(self.cfg)
        for _ in range(10):
            gain, action = g.decide(1.0, 0.02, 0.5, mode="nn")
        self.assertEqual(action, "slow")
        self.assertGreaterEqual(gain, 1.0)

    def test_ema_damps_single_spike(self):
        g = FaultGuard(self.cfg)
        self.assertLess(g.smooth(1.0), self.cfg.guard_brake_risk)

    def test_reset_clears_ema(self):
        g = FaultGuard(self.cfg)
        for _ in range(5):
            g.smooth(1.0)
        g.reset()
        self.assertEqual(g._risk_ema, 0.0)


class TestRuleGate(unittest.TestCase):
    def test_small_difference_passes(self):
        thr, action = rule_gate(0.05, 0.5, 1.0)
        self.assertEqual(action, "pass")
        self.assertAlmostEqual(thr, 1.0)

    def test_mid_difference_assists(self):
        thr, action = rule_gate(0.3, 0.5, 1.0)
        self.assertEqual(action, "assist")
        self.assertLess(thr, 1.0)

    def test_large_difference_slows(self):
        thr, action = rule_gate(0.9, 0.5, 1.0)
        self.assertEqual(action, "slow")


if __name__ == "__main__":
    unittest.main()
