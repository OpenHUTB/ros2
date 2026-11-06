"""模型公式的解析校验。"""

import unittest

import numpy as np

from tire_friction_braking_sweep.model import (
    GRAVITY,
    BrakingSpec,
    bottleneck,
    braking_distance_m,
    friction_bound_decel_mps2,
    ideal_decel_mps2,
    saturated_decel_mps2,
)


class ModelTest(unittest.TestCase):
    def test_friction_bound_is_mu_g(self):
        self.assertAlmostEqual(float(friction_bound_decel_mps2(0.8)), 0.8 * GRAVITY, places=9)
        self.assertTrue(np.allclose(friction_bound_decel_mps2([0.4, 1.0]), [0.4 * GRAVITY, 1.0 * GRAVITY]))

    def test_ideal_takes_the_smaller_limit(self):
        brake_limit = 6.0
        self.assertAlmostEqual(float(ideal_decel_mps2(0.4, brake_limit)), 0.4 * GRAVITY, places=9)
        self.assertAlmostEqual(float(ideal_decel_mps2(1.2, brake_limit)), brake_limit, places=9)

    def test_braking_distance_matches_analytic(self):
        speed = 50.0 / 3.6
        distance = float(braking_distance_m(speed, 7.0))
        self.assertAlmostEqual(distance, speed**2 / 14.0, places=9)

    def test_distance_scales_with_speed_squared(self):
        speed = 20.0
        base = float(braking_distance_m(speed, 5.0))
        self.assertAlmostEqual(float(braking_distance_m(2 * speed, 5.0)), 4.0 * base, places=9)

    def test_saturation_curve_shape(self):
        a_max, mu_half = 6.4, 0.5
        self.assertAlmostEqual(float(saturated_decel_mps2(mu_half, a_max, mu_half)), a_max / 2, places=9)
        # 附着系数很大时趋近 a_max
        self.assertGreater(float(saturated_decel_mps2(50.0, a_max, mu_half)), 0.98 * a_max)
        # 单调递增
        values = saturated_decel_mps2(np.linspace(0.2, 3.0, 50), a_max, mu_half)
        self.assertTrue(np.all(np.diff(values) > 0.0))

    def test_bottleneck_switches_at_threshold(self):
        self.assertEqual(bottleneck(0.4, 6.4), "附着受限")
        self.assertEqual(bottleneck(1.5, 6.4), "制动器受限")

    def test_spec_reference_speed(self):
        spec = BrakingSpec(reference_speed_kmh=72.0)
        self.assertAlmostEqual(spec.reference_speed_mps, 20.0, places=9)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            friction_bound_decel_mps2(-0.1)
        with self.assertRaises(ValueError):
            ideal_decel_mps2(0.8, 0.0)
        with self.assertRaises(ValueError):
            braking_distance_m(10.0, 0.0)
        with self.assertRaises(ValueError):
            braking_distance_m(-1.0, 5.0)
        with self.assertRaises(ValueError):
            saturated_decel_mps2(0.8, 6.4, 0.0)
        with self.assertRaises(ValueError):
            bottleneck(0.8, 6.4, tolerance=1.5)
        with self.assertRaises(ValueError):
            BrakingSpec(mass_kg=0.0).validate()
        with self.assertRaises(ValueError):
            BrakingSpec(brake_ratio=1.5).validate()


if __name__ == "__main__":
    unittest.main()
