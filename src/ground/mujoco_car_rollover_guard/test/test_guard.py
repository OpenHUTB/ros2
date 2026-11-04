# -*- coding: utf-8 -*-
"""风险门控与反应式门限测试（不依赖 MuJoCo）。"""
import unittest

from mujoco_car_rollover_guard.config import Config
from mujoco_car_rollover_guard.guard import RolloverGuard, rule_gate


class TestRolloverGuard(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()

    def test_low_risk_passes_command_unchanged(self):
        g = RolloverGuard(self.cfg)
        steer, thr, action = g.gate(0.1, 0.5, 0.8)
        self.assertEqual(action, "pass")
        self.assertAlmostEqual(steer, 0.5)
        self.assertAlmostEqual(thr, 0.8)

    def test_middle_risk_slows_down(self):
        g = RolloverGuard(self.cfg)
        _, thr, action = g.gate(0.75, 0.3, 1.0, smooth=False)
        self.assertEqual(action, "slow")
        self.assertLess(thr, 1.0)

    def test_high_risk_brakes(self):
        g = RolloverGuard(self.cfg)
        _, thr, action = g.gate(0.95, 0.3, 1.0, smooth=False)
        self.assertEqual(action, "brake")
        self.assertEqual(thr, 0.0)

    def test_ema_damps_single_spike(self):
        g = RolloverGuard(self.cfg)
        r = g.smooth(1.0)
        self.assertLess(r, self.cfg.guard_brake_risk)

    def test_ema_reaches_high_risk_after_repeats(self):
        g = RolloverGuard(self.cfg)
        for _ in range(8):
            r = g.smooth(1.0)
        self.assertGreater(r, self.cfg.guard_brake_risk)

    def test_reset_clears_ema(self):
        g = RolloverGuard(self.cfg)
        for _ in range(5):
            g.smooth(1.0)
        g.reset()
        self.assertEqual(g._risk_ema, 0.0)


class TestRuleGate(unittest.TestCase):
    def test_small_roll_passes(self):
        thr, action = rule_gate(10.0, 1.0)
        self.assertEqual(action, "pass")
        self.assertAlmostEqual(thr, 1.0)

    def test_mid_roll_slows(self):
        thr, action = rule_gate(27.5, 1.0)
        self.assertEqual(action, "slow")
        self.assertLess(thr, 1.0)

    def test_large_roll_brakes(self):
        thr, action = rule_gate(40.0, 1.0)
        self.assertEqual(action, "brake")
        self.assertEqual(thr, 0.0)


if __name__ == "__main__":
    unittest.main()
