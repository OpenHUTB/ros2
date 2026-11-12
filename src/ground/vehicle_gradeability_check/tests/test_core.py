import math
import unittest

from vehicle_gradeability_check.core import (
    GradeabilityChecker,
    GradeabilityConfig,
    required_force_n,
    traction_limit_n,
)
from vehicle_gradeability_check.demo import run_demo


class GradeabilityTests(unittest.TestCase):
    def test_flat_slope_has_no_grade_component(self):
        config = GradeabilityConfig()
        flat = required_force_n(config, 0.0, 0.0)
        expected_rolling = (
            config.mass_kg * config.gravity * config.crr
        )
        self.assertAlmostEqual(flat, expected_rolling, places=3)

    def test_steeper_slope_requires_larger_force(self):
        config = GradeabilityConfig()
        self.assertGreater(
            required_force_n(config, 15.0, 5.0),
            required_force_n(config, 5.0, 5.0),
        )

    def test_traction_limit_drops_with_slope(self):
        config = GradeabilityConfig()
        self.assertGreater(
            traction_limit_n(config, 0.0), traction_limit_n(config, 25.0)
        )

    def test_gentle_slope_is_climbable(self):
        checker = GradeabilityChecker()
        checker.update_slope(5.0, 0.0)
        checker.update_speed(6.0, 0.0)
        result = checker.step(0.05)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "CLIMBABLE")
        self.assertFalse(result.slip_risk)
        self.assertFalse(result.power_deficit)
        self.assertGreaterEqual(result.margin_n, 0.0)

    def test_steep_slope_reports_power_deficit(self):
        checker = GradeabilityChecker(
            GradeabilityConfig(max_drive_force_n=4500.0)
        )
        checker.update_slope(24.0, 0.0)
        checker.update_speed(5.0, 0.0)
        result = checker.step(0.05)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "POWER_LIMIT")
        self.assertTrue(result.power_deficit)
        self.assertLess(result.margin_n, 0.0)

    def test_low_friction_reports_traction_limit(self):
        checker = GradeabilityChecker(
            GradeabilityConfig(friction_coef=0.25, max_drive_force_n=20000.0)
        )
        checker.update_slope(20.0, 0.0)
        checker.update_speed(5.0, 0.0)
        result = checker.step(0.05)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "TRACTION_LIMIT")
        self.assertTrue(result.slip_risk)

    def test_slope_timeout_holds_stale_state(self):
        checker = GradeabilityChecker(GradeabilityConfig(slope_timeout_s=0.5))
        checker.update_slope(10.0, 1.0)
        checker.update_speed(6.0, 1.0)
        result = checker.step(1.6)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "SLOPE_STALE")
        self.assertEqual(result.margin_n, 0.0)

    def test_speed_timeout_holds_stale_state(self):
        checker = GradeabilityChecker(GradeabilityConfig(speed_timeout_s=0.5))
        checker.update_slope(10.0, 1.0)
        checker.update_speed(6.0, 1.0)
        result = checker.step(1.6)
        self.assertIsNotNone(result)
        self.assertEqual(result.mode, "SPEED_STALE")
        self.assertEqual(result.margin_n, 0.0)

    def test_refreshing_input_prevents_stale_state(self):
        checker = GradeabilityChecker(GradeabilityConfig(slope_timeout_s=0.5))
        checker.update_slope(10.0, 1.0)
        checker.update_slope(10.0, 1.6)
        checker.update_speed(6.0, 1.6)
        result = checker.step(1.6)
        self.assertIsNotNone(result)
        self.assertNotEqual(result.mode, "SLOPE_STALE")

    def test_invalid_input_keeps_last_valid_value(self):
        checker = GradeabilityChecker()
        checker.update_slope(10.0, 0.0)
        with self.assertRaises(ValueError):
            checker.update_slope(math.nan, 0.1)
        self.assertAlmostEqual(checker.slope_deg, 10.0)

    def test_invalid_configuration_is_rejected(self):
        for kwargs in (
            {"mass_kg": 0.0},
            {"gravity": -1.0},
            {"crr": math.nan},
            {"cda": -0.1},
            {"air_density": 0.0},
            {"max_drive_force_n": -1.0},
            {"friction_coef": 0.0},
            {"slope_timeout_s": math.inf},
            {"speed_timeout_s": -1.0},
            {"control_rate_hz": 0.0},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    GradeabilityConfig(**kwargs)

    def test_offline_demo_reports_deficit_and_climbable_slope(self):
        _, summary = run_demo(duration_s=10.0, rate_hz=20.0)
        self.assertEqual(summary["samples"], 201)
        self.assertGreater(summary["power_limit_samples"], 0)
        self.assertGreater(summary["climbable_samples"], 0)
        self.assertLess(summary["min_margin_n"], 0.0)
        self.assertGreater(summary["max_climbable_slope_deg"], 0.0)


if __name__ == "__main__":
    unittest.main()
