# -*- coding: utf-8 -*-
"""信号灯决策与纵向控制器的单元测试。

这两部分不依赖 CARLA，可以完整跑单元测试。
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from longitudinal import (  # noqa: E402
    ControllerState,
    LongitudinalController,
)
from traffic_light_logic import (  # noqa: E402
    GO,
    GREEN,
    OFF,
    RED,
    SLOW,
    STOP,
    LightParams,
    YELLOW,
    braking_distance,
    decide,
    required_deceleration,
    target_speed_for_stop,
)


# ===========================================================================
class TestBrakingDistance(unittest.TestCase):
    def test_formula(self):
        # 10 m/s, 5 m/s² → 100/10 = 10 m
        self.assertAlmostEqual(braking_distance(10.0, 5.0), 10.0, places=12)

    def test_zero_speed_zero_distance(self):
        self.assertAlmostEqual(braking_distance(0.0, 5.0), 0.0, places=12)

    def test_quadratic_in_speed(self):
        a = 5.0
        self.assertAlmostEqual(braking_distance(20.0, a),
                               4.0 * braking_distance(10.0, a), places=9)

    def test_inverse_in_deceleration(self):
        v = 12.0
        self.assertAlmostEqual(braking_distance(v, 4.0),
                               2.0 * braking_distance(v, 8.0), places=9)

    def test_negative_speed_uses_absolute(self):
        self.assertAlmostEqual(braking_distance(-10.0, 5.0),
                               braking_distance(10.0, 5.0), places=12)

    def test_reject_non_positive_deceleration(self):
        with self.assertRaises(ValueError):
            braking_distance(10.0, 0.0)
        with self.assertRaises(ValueError):
            braking_distance(10.0, -1.0)


# ===========================================================================
class TestRequiredDeceleration(unittest.TestCase):
    def test_inverse_of_braking_distance(self):
        v, d = 15.0, 20.0
        a = required_deceleration(v, d)
        self.assertAlmostEqual(braking_distance(v, a), d, places=9)

    def test_infinite_when_no_distance(self):
        self.assertEqual(required_deceleration(10.0, 0.0), float("inf"))

    def test_smaller_distance_needs_more_deceleration(self):
        self.assertGreater(required_deceleration(10.0, 5.0),
                           required_deceleration(10.0, 20.0))


# ===========================================================================
class TestTargetSpeed(unittest.TestCase):
    def test_zero_at_stop_line(self):
        self.assertAlmostEqual(target_speed_for_stop(0.0, 2.0), 0.0, places=12)

    def test_formula(self):
        # sqrt(2 * 2 * 16) = 8
        self.assertAlmostEqual(target_speed_for_stop(16.0, 2.0), 8.0, places=9)

    def test_sqrt_shape(self):
        a = 2.0
        self.assertLess(target_speed_for_stop(4.0, a),
                        target_speed_for_stop(9.0, a))

    def test_reject_bad_deceleration(self):
        with self.assertRaises(ValueError):
            target_speed_for_stop(10.0, 0.0)


# ===========================================================================
class TestLightParams(unittest.TestCase):
    def test_defaults(self):
        p = LightParams()
        self.assertGreater(p.comfort_decel, 0)
        self.assertGreaterEqual(p.max_decel, p.comfort_decel)

    def test_reject_bad_values(self):
        with self.assertRaises(ValueError):
            LightParams(comfort_decel=0.0)
        with self.assertRaises(ValueError):
            LightParams(max_decel=0.0)
        with self.assertRaises(ValueError):
            LightParams(yellow_min_gap=-1.0)
        with self.assertRaises(ValueError):
            LightParams(stop_margin=-1.0)

    def test_max_decel_not_less_than_comfort(self):
        with self.assertRaises(ValueError):
            LightParams(comfort_decel=5.0, max_decel=3.0)


# ===========================================================================
class TestDecideGreen(unittest.TestCase):
    def test_green_go(self):
        action, v, _ = decide(GREEN, 30.0, 10.0)
        self.assertEqual(action, GO)
        self.assertIsNone(v)

    def test_off_go(self):
        action, _v, _ = decide(OFF, 30.0, 10.0)
        self.assertEqual(action, GO)

    def test_green_near_stop_line_still_go(self):
        action, _v, _ = decide(GREEN, 1.0, 10.0)
        self.assertEqual(action, GO)


# ===========================================================================
class TestDecideRed(unittest.TestCase):
    def test_red_far_away_stops_with_profile(self):
        action, v_t, _ = decide(RED, 30.0, 12.0)
        self.assertEqual(action, STOP)
        self.assertIsNotNone(v_t)
        self.assertGreater(v_t, 0.0)     # 还远，只是按曲线减速
        self.assertLess(v_t, 12.0)

    def test_red_at_stop_line_commands_zero(self):
        action, v_t, _ = decide(RED, 0.0, 3.0)
        self.assertEqual(action, STOP)
        self.assertAlmostEqual(v_t, 0.0, places=9)

    def test_red_too_close_commands_zero(self):
        # 速度 20 m/s，最大减速 6 → 需要 33 m，但只剩 10 m
        action, v_t, note = decide(RED, 10.0, 20.0)
        self.assertEqual(action, STOP)
        self.assertAlmostEqual(v_t, 0.0, places=9)
        self.assertIn("停不住", note)

    def test_red_target_speed_decreases_as_distance_shrinks(self):
        _a1, v1, _ = decide(RED, 40.0, 10.0)
        _a2, v2, _ = decide(RED, 20.0, 10.0)
        _a3, v3, _ = decide(RED, 10.0, 10.0)
        self.assertGreater(v1, v2)
        self.assertGreater(v2, v3)

    def test_passed_stop_line_ignores_red(self):
        action, _v, note = decide(RED, -2.0, 8.0)
        self.assertEqual(action, GO)
        self.assertIn("越过", note)

    def test_passed_stop_line_flag(self):
        action, _v, _ = decide(RED, 5.0, 8.0, passed_stop_line=True)
        self.assertEqual(action, GO)


# ===========================================================================
class TestDecideYellow(unittest.TestCase):
    def test_yellow_can_stop_returns_slow(self):
        p = LightParams(comfort_decel=2.0, max_decel=6.0)
        # 10 m/s 停住需 100/12 ≈ 8.33 m，距离 30 m 足够
        action, v_t, note = decide(YELLOW, 30.0, 10.0, p)
        self.assertEqual(action, SLOW)
        self.assertIsNotNone(v_t)
        self.assertIn("可停住", note)

    def test_yellow_cannot_stop_returns_go(self):
        p = LightParams(comfort_decel=2.0, max_decel=6.0)
        # 20 m/s 停住需 400/12 ≈ 33.3 m，只剩 10 m
        action, _v, note = decide(YELLOW, 10.0, 20.0, p)
        self.assertEqual(action, GO)
        self.assertIn("保持通过", note)

    def test_yellow_very_close_always_go(self):
        p = LightParams(yellow_min_gap=2.0)
        action, _v, note = decide(YELLOW, 1.0, 5.0, p)
        self.assertEqual(action, GO)
        self.assertIn("通过", note)

    def test_yellow_low_speed_stops(self):
        p = LightParams(min_speed=0.5)
        action, v_t, _ = decide(YELLOW, 8.0, 0.2, p)
        self.assertEqual(action, STOP)
        self.assertAlmostEqual(v_t, 0.0, places=9)

    def test_yellow_boundary_condition(self):
        """刚好能停住的距离应当判为可停。"""
        p = LightParams(comfort_decel=2.0, max_decel=6.0, stop_margin=0.0,
                        yellow_min_gap=0.0)
        v = 10.0
        d_exact = braking_distance(v, p.max_decel)
        action, _vt, _n = decide(YELLOW, d_exact, v, p)
        self.assertEqual(action, SLOW)

    def test_yellow_just_below_boundary_goes(self):
        p = LightParams(comfort_decel=2.0, max_decel=6.0, stop_margin=0.0,
                        yellow_min_gap=0.0)
        v = 10.0
        d_short = braking_distance(v, p.max_decel) - 0.5
        action, _vt, _n = decide(YELLOW, d_short, v, p)
        self.assertEqual(action, GO)


# ===========================================================================
class TestDecideValidation(unittest.TestCase):
    def test_reject_unknown_state(self):
        with self.assertRaises(ValueError):
            decide("Purple", 10.0, 5.0)

    def test_negative_distance_treated_as_passed(self):
        action, _v, _n = decide(RED, -5.0, 5.0)
        self.assertEqual(action, GO)


# ===========================================================================
class TestLongitudinalController(unittest.TestCase):
    def test_zero_error_no_output(self):
        c = LongitudinalController(deadband=0.0)
        th, br = c.update(10.0, 10.0, 0.05)
        self.assertAlmostEqual(th, 0.0, places=9)
        self.assertAlmostEqual(br, 0.0, places=9)

    def test_too_slow_gives_throttle(self):
        c = LongitudinalController()
        th, br = c.update(5.0, 10.0, 0.05)
        self.assertGreater(th, 0.0)
        self.assertAlmostEqual(br, 0.0, places=9)

    def test_too_fast_gives_brake(self):
        c = LongitudinalController()
        th, br = c.update(15.0, 10.0, 0.05)
        self.assertAlmostEqual(th, 0.0, places=9)
        self.assertGreater(br, 0.0)

    def test_outputs_clamped_to_unit(self):
        c = LongitudinalController(kp=50.0, ki=50.0)
        for _ in range(200):
            th, br = c.update(0.0, 50.0, 0.05)
            self.assertGreaterEqual(th, 0.0)
            self.assertLessEqual(th, 1.0)
            self.assertGreaterEqual(br, 0.0)
            self.assertLessEqual(br, 1.0)

    def test_stop_command_brakes_hard(self):
        c = LongitudinalController()
        th, br = c.update(0.1, 0.0, 0.05)
        self.assertAlmostEqual(th, 0.0, places=9)
        self.assertAlmostEqual(br, 1.0, places=9)

    def test_integral_limited(self):
        c = LongitudinalController(ki=10.0, integral_max=2.0)
        for _ in range(500):
            c.update(0.0, 20.0, 0.05)
        self.assertLessEqual(abs(c.state.integral), 2.0 + 1e-9)

    def test_deadband_suppresses_small_error(self):
        c = LongitudinalController(deadband=0.5)
        th, br = c.update(10.0, 10.2, 0.05)
        self.assertAlmostEqual(th, 0.0, places=9)
        self.assertAlmostEqual(br, 0.0, places=9)

    def test_reset_clears_state(self):
        c = LongitudinalController()
        for _ in range(20):
            c.update(0.0, 10.0, 0.05)
        c.reset()
        self.assertAlmostEqual(c.state.integral, 0.0)
        self.assertAlmostEqual(c.state.throttle, 0.0)

    def test_reject_bad_dt(self):
        c = LongitudinalController()
        with self.assertRaises(ValueError):
            c.update(0.0, 10.0, 0.0)
        with self.assertRaises(ValueError):
            c.update(0.0, 10.0, -0.1)

    def test_reject_bad_params(self):
        with self.assertRaises(ValueError):
            LongitudinalController(kp=-1.0)
        with self.assertRaises(ValueError):
            LongitudinalController(ki=-1.0)
        with self.assertRaises(ValueError):
            LongitudinalController(throttle_gain=0.0)
        with self.assertRaises(ValueError):
            LongitudinalController(deadband=-0.1)

    def test_converges_toward_target(self):
        """简单积分模型下应逐步接近目标速度。"""
        c = LongitudinalController(kp=1.0, ki=0.2)
        v = 0.0
        target = 8.0
        dt = 0.05
        for _ in range(600):
            th, br = c.update(v, target, dt)
            a = 3.0 * th - 6.0 * br
            v = max(0.0, v + a * dt)
        self.assertAlmostEqual(v, target, delta=1.0)

    def test_brakes_to_stop(self):
        c = LongitudinalController(kp=1.0, ki=0.2)
        v = 10.0
        dt = 0.05
        for _ in range(1000):
            th, br = c.update(v, 0.0, dt)
            a = 3.0 * th - 6.0 * br
            v = max(0.0, v + a * dt)
            if v <= 1e-6:
                break
        self.assertLess(v, 0.5)

    def test_state_object_updated(self):
        c = LongitudinalController()
        c.update(5.0, 10.0, 0.05)
        self.assertIsInstance(c.state, ControllerState)
        self.assertNotAlmostEqual(c.state.a_cmd, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
