import math
import unittest

from uav_yaw_command_guard.core import (
    YawGuard,
    YawGuardConfig,
    shortest_angle,
    wrap_angle,
)
from uav_yaw_command_guard.demo import run_demo


class YawGuardTests(unittest.TestCase):
    def test_wrap_angle_uses_half_open_interval(self):
        self.assertAlmostEqual(wrap_angle(math.radians(181.0)), math.radians(-179.0))
        self.assertAlmostEqual(wrap_angle(math.radians(-181.0)), math.radians(179.0))
        self.assertAlmostEqual(wrap_angle(math.pi), -math.pi)

    def test_wrap_crossing_selects_two_degree_short_path(self):
        self.assertAlmostEqual(
            shortest_angle(math.radians(-179.0), math.radians(179.0)),
            math.radians(2.0),
        )
        self.assertAlmostEqual(
            shortest_angle(math.radians(179.0), math.radians(-179.0)),
            math.radians(-2.0),
        )

    def test_guard_returns_none_without_current_yaw(self):
        guard = YawGuard()
        guard.update_target(0.0, 0.0)
        self.assertIsNone(guard.step(0.0))

    def test_large_yaw_error_is_rate_limited(self):
        guard = YawGuard(YawGuardConfig(max_yaw_rate_deg_s=90.0))
        guard.update_current(0.0, 0.0)
        guard.update_target(math.radians(170.0), 0.0)
        result = guard.step(0.05)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "LIMITED")
        self.assertTrue(result.limited)
        self.assertAlmostEqual(
            result.yaw_rate_rad_s, math.radians(90.0), places=9
        )

    def test_small_wrap_error_is_not_rate_limited(self):
        guard = YawGuard()
        guard.update_current(math.radians(179.0), 0.0)
        guard.update_target(math.radians(-179.0), 0.0)
        result = guard.step(0.05)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "TRACKING")
        self.assertFalse(result.limited)
        self.assertAlmostEqual(result.error_rad, math.radians(2.0), places=9)
        self.assertAlmostEqual(
            result.yaw_rate_rad_s, math.radians(40.0), places=9
        )

    def test_target_timeout_holds_zero_rate(self):
        guard = YawGuard(YawGuardConfig(target_timeout_s=0.5))
        guard.update_current(0.0, 1.0)
        guard.update_target(math.radians(90.0), 1.0)
        result = guard.step(1.6)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "HOLD")
        self.assertEqual(result.error_rad, 0.0)
        self.assertEqual(result.yaw_rate_rad_s, 0.0)

    def test_current_timeout_holds_zero_rate(self):
        guard = YawGuard(YawGuardConfig(current_timeout_s=0.2))
        guard.update_current(0.0, 0.0)
        guard.update_target(math.radians(90.0), 0.3)
        result = guard.step(0.3)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "CURRENT_STALE")
        self.assertEqual(result.error_rad, 0.0)
        self.assertEqual(result.yaw_rate_rad_s, 0.0)

    def test_refreshing_current_prevents_stale_hold(self):
        guard = YawGuard(YawGuardConfig(current_timeout_s=0.2))
        guard.update_current(0.0, 0.0)
        guard.update_current(0.0, 0.3)
        guard.update_target(math.radians(90.0), 0.3)
        result = guard.step(0.3)
        self.assertIsNotNone(result)
        self.assertNotEqual(result.mode, "CURRENT_STALE")
        self.assertGreater(result.yaw_rate_rad_s, 0.0)

    def test_invalid_input_does_not_replace_last_valid_value(self):
        guard = YawGuard()
        guard.update_current(1.0, 0.0)
        with self.assertRaises(ValueError):
            guard.update_current(math.nan, 0.1)
        self.assertAlmostEqual(guard.current_yaw, 1.0)

    def test_invalid_configuration_is_rejected(self):
        for kwargs in (
            {"control_rate_hz": 0.0},
            {"control_rate_hz": math.inf},
            {"max_yaw_rate_deg_s": -1.0},
            {"max_yaw_rate_deg_s": math.nan},
            {"target_timeout_s": 0.0},
            {"current_timeout_s": -1.0},
            {"current_timeout_s": math.inf},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    YawGuardConfig(**kwargs)

    def test_offline_demo_respects_rate_limit_and_reaches_final_target(self):
        _, summary = run_demo(duration_s=8.0, rate_hz=20.0)
        self.assertEqual(summary["samples"], 161)
        self.assertEqual(summary["max_abs_rate_deg_s"], 90.0)
        self.assertEqual(summary["final_error_deg"], 0.0)
        self.assertGreater(summary["limited_samples"], 0)


if __name__ == "__main__":
    unittest.main()
