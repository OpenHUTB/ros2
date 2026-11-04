# -*- coding: utf-8 -*-
"""决策状态机单测（不需要 MuJoCo）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mujoco_car_avoidance.decision import DecisionParams, State, decide  # noqa: E402


class TestDecisionParams(unittest.TestCase):
    """阈值合法性。"""

    def test_default_is_valid(self):
        params = DecisionParams()
        self.assertLess(params.emergency_m, params.stop_target_m)
        self.assertLess(params.stop_target_m, params.slow_start_m)

    def test_inconsistent_thresholds_raise(self):
        # 急刹区比期望停车位置还远，属于自相矛盾的配置
        with self.assertRaises(ValueError):
            DecisionParams(stop_target_m=1.0, emergency_m=2.0, slow_start_m=6.0)

    def test_emergency_equals_stop_target_raises(self):
        with self.assertRaises(ValueError):
            DecisionParams(stop_target_m=2.0, emergency_m=2.0, slow_start_m=6.0)

    def test_non_positive_cruise_speed_raises(self):
        with self.assertRaises(ValueError):
            DecisionParams(cruise_speed=0.0)


class TestDecide(unittest.TestCase):
    """状态与目标速度。"""

    def setUp(self):
        self.params = DecisionParams(cruise_speed=2.0, slow_start_m=6.0,
                                     stop_target_m=1.8, emergency_m=0.8,
                                     stale_timeout_s=0.5)

    def test_no_measurement_fresh_keeps_standstill(self):
        result = decide(None, 0.0, self.params)
        self.assertEqual(result.state, State.FULL_BRAKE)
        self.assertEqual(result.target_speed, 0.0)

    def test_no_measurement_timeout_is_stale(self):
        result = decide(None, 1.0, self.params)
        self.assertEqual(result.state, State.STALE)
        self.assertEqual(result.target_speed, 0.0)
        self.assertIn("兜底", result.reason)

    def test_far_obstacle_cruises(self):
        result = decide(15.0, 0.0, self.params)
        self.assertEqual(result.state, State.CRUISE)
        self.assertAlmostEqual(result.target_speed, self.params.cruise_speed)

    def test_slow_zone_interpolates(self):
        # 减速区中点应给出约一半巡航速度
        middle = (self.params.slow_start_m + self.params.stop_target_m) / 2.0
        result = decide(middle, 0.0, self.params)
        self.assertEqual(result.state, State.SLOW_DOWN)
        self.assertAlmostEqual(result.target_speed, self.params.cruise_speed / 2.0, places=6)

    def test_slow_zone_is_monotonic(self):
        distances = [6.0, 5.0, 4.0, 3.0, 2.5, 1.9]
        speeds = [decide(d, 0.0, self.params).target_speed for d in distances]
        self.assertEqual(speeds, sorted(speeds, reverse=True))

    def test_slow_zone_bounds(self):
        at_start = decide(self.params.slow_start_m, 0.0, self.params)
        at_target = decide(self.params.stop_target_m, 0.0, self.params)
        self.assertAlmostEqual(at_start.target_speed, self.params.cruise_speed, places=6)
        self.assertAlmostEqual(at_target.target_speed, 0.0, places=6)

    def test_emergency_zone_brakes(self):
        result = decide(self.params.emergency_m, 0.0, self.params)
        self.assertEqual(result.state, State.FULL_BRAKE)
        self.assertEqual(result.target_speed, 0.0)

    def test_below_creep_speed_brakes_firmly(self):
        # 距停车目标仅 5 cm 时，线性插值出的目标速度低于蠕动阈值(0.12 m/s)，
        # 应直接进入主动刹停，而不是继续低速蠕动。
        result = decide(self.params.stop_target_m + 0.05, 0.0, self.params)
        self.assertEqual(result.state, State.FULL_BRAKE)
        self.assertEqual(result.target_speed, 0.0)
        self.assertIn("蠕动", result.reason)

    def test_very_close_brakes(self):
        result = decide(0.3, 0.0, self.params)
        self.assertEqual(result.state, State.FULL_BRAKE)

    def test_target_speed_never_exceeds_cruise(self):
        for distance in [0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 6.0, 100.0]:
            result = decide(distance, 0.0, self.params)
            self.assertLessEqual(result.target_speed, self.params.cruise_speed + 1e-9)
            self.assertGreaterEqual(result.target_speed, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
