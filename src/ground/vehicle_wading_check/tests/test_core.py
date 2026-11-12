import unittest

from vehicle_wading_check.core import (
    WadingChecker,
    WadingConfig,
    buoyancy_force,
    evaluate_depth,
    float_depth,
)


class TestGeometryLimits(unittest.TestCase):
    def test_depth_limit_is_lowest_opening(self):
        config = WadingConfig()
        result = evaluate_depth(0.05, 0.0, config)
        self.assertAlmostEqual(result.depth_limit_m, 0.35, places=6)
        self.assertAlmostEqual(result.safe_depth_m, 0.28, places=6)

    def test_flotation_depth_matches_volume_model(self):
        config = WadingConfig()
        expected = 0.9 * config.mass_kg / (
            config.water_density * config.body_length_m
            * config.body_width_m * config.displacement_volume_factor
        )
        self.assertAlmostEqual(float_depth(config), expected, places=6)
        self.assertAlmostEqual(float_depth(config), 0.4762, places=4)


class TestModes(unittest.TestCase):
    def test_safe_below_safe_depth(self):
        result = evaluate_depth(0.20, 2.0, WadingConfig())
        self.assertEqual(result.mode, "SAFE")

    def test_caution_between_safe_and_limit(self):
        result = evaluate_depth(0.30, 2.0, WadingConfig())
        self.assertEqual(result.mode, "CAUTION")

    def test_flooded_above_limit(self):
        result = evaluate_depth(0.40, 2.0, WadingConfig())
        self.assertEqual(result.mode, "FLOODED")

    def test_floating_near_flotation_depth(self):
        result = evaluate_depth(0.48, 2.0, WadingConfig())
        self.assertEqual(result.mode, "FLOATING")
        self.assertEqual(result.v_max_mps, 0.0)


class TestForces(unittest.TestCase):
    def test_buoyancy_grows_with_depth(self):
        config = WadingConfig()
        self.assertAlmostEqual(buoyancy_force(0.0, config), 0.0, places=6)
        self.assertLess(
            buoyancy_force(0.10, config), buoyancy_force(0.25, config)
        )

    def test_buoyancy_saturates_at_body_height(self):
        config = WadingConfig()
        self.assertAlmostEqual(
            buoyancy_force(2.0, config), buoyancy_force(config.body_height_m, config),
            places=6,
        )

    def test_traction_limit_drops_as_water_rises(self):
        config = WadingConfig()
        dry = evaluate_depth(0.02, 0.0, config)
        deep = evaluate_depth(0.30, 0.0, config)
        self.assertLess(deep.traction_limit_n, dry.traction_limit_n)

    def test_allowed_speed_decreases_with_depth(self):
        config = WadingConfig()
        shallow = evaluate_depth(0.05, 0.0, config).v_max_mps
        deeper = evaluate_depth(0.25, 0.0, config).v_max_mps
        self.assertLess(deeper, shallow)
        self.assertGreater(deeper, 0.0)

    def test_drag_quadratic_in_speed(self):
        config = WadingConfig()
        slow = evaluate_depth(0.20, 1.0, config)
        fast = evaluate_depth(0.20, 2.0, config)
        self.assertAlmostEqual(
            fast.drag_n, 4.0 * slow.drag_n, delta=0.05 * fast.drag_n
        )

    def test_margin_negative_when_over_speed_limit(self):
        result = evaluate_depth(0.42, 2.5, WadingConfig())
        self.assertGreater(result.speed_mps, result.v_max_mps)
        self.assertLess(result.traction_margin_n, 0.0)

    def test_brake_decay_from_wet_friction(self):
        config = WadingConfig()
        result = evaluate_depth(0.20, 1.0, config)
        self.assertAlmostEqual(
            result.brake_decay_ratio, config.friction_coef_wet / config.friction_coef_dry,
            places=6,
        )


class TestChecker(unittest.TestCase):
    def test_returns_none_until_both_inputs_arrive(self):
        checker = WadingChecker()
        self.assertIsNone(checker.step(0.0))
        checker.update_depth(0.2, 0.0)
        self.assertIsNone(checker.step(0.1))
        checker.update_speed(1.0, 0.1)
        self.assertIsNotNone(checker.step(0.2))

    def test_stale_depth_blocks_result(self):
        checker = WadingChecker(WadingConfig(depth_timeout_s=0.5))
        checker.update_depth(0.2, 0.0)
        checker.update_speed(1.0, 0.0)
        self.assertIsNone(checker.step(1.0))

    def test_reset_clears_freshness(self):
        checker = WadingChecker()
        checker.update_depth(0.2, 0.0)
        checker.update_speed(1.0, 0.0)
        self.assertIsNotNone(checker.step(0.1))
        checker.reset()
        self.assertIsNone(checker.step(0.1))

    def test_rejects_invalid_input(self):
        config = WadingConfig()
        with self.assertRaises(ValueError):
            evaluate_depth(-0.1, 1.0, config)
        with self.assertRaises(ValueError):
            WadingConfig(mass_kg=-1.0)
        with self.assertRaises(ValueError):
            WadingConfig(safety_margin_ratio=1.5)


if __name__ == "__main__":
    unittest.main()
