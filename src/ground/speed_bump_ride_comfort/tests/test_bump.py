"""减速带几何的解析校验。"""

import unittest

import numpy as np

from speed_bump_ride_comfort.bump import KINDS, SpeedBump


class SpeedBumpGeometryTest(unittest.TestCase):
    def test_half_sine_profile_hits_peak_at_midpoint(self):
        bump = SpeedBump(height_m=0.06, width_m=1.2, kind="half_sine")
        self.assertAlmostEqual(float(bump.profile(0.0)), 0.0)
        self.assertAlmostEqual(float(bump.profile(0.6)), 0.06, places=9)
        self.assertAlmostEqual(float(bump.profile(1.2)), 0.0)

    def test_profile_is_zero_outside_the_bump(self):
        bump = SpeedBump()
        samples = bump.profile(np.array([-3.0, -0.001, 1.201, 5.0]))
        self.assertTrue(np.allclose(samples, 0.0))

    def test_half_sine_area_matches_analytic_value(self):
        bump = SpeedBump(height_m=0.06, width_m=1.2, kind="half_sine")
        x = np.linspace(0.0, bump.width_m, 200001)
        numeric = float(np.trapezoid(bump.profile(x), x))
        self.assertAlmostEqual(numeric, bump.reference_area_m2(), places=6)

    def test_trapezoid_has_ramps_and_plateau(self):
        bump = SpeedBump(height_m=0.08, width_m=2.0, kind="trapezoid", ramp_ratio=0.25)
        self.assertAlmostEqual(float(bump.profile(0.25)), 0.04, places=9)  # 半坡
        self.assertAlmostEqual(float(bump.profile(1.00)), 0.08, places=9)  # 平台
        self.assertAlmostEqual(float(bump.profile(1.75)), 0.04, places=9)  # 下坡一半
        x = np.linspace(0.0, bump.width_m, 200001)
        numeric = float(np.trapezoid(bump.profile(x), x))
        self.assertAlmostEqual(numeric, bump.reference_area_m2(), places=6)

    def test_profile_time_follows_the_vehicle_speed(self):
        bump = SpeedBump()
        times = np.array([-0.1, 0.0, 0.15, 0.3, 0.6])
        for speed in (1.5, 4.0, 9.0):
            self.assertTrue(
                np.allclose(bump.profile_time(times, speed), bump.profile(times * speed))
            )

    def test_contact_duration_scales_with_speed(self):
        bump = SpeedBump(width_m=1.2)
        self.assertAlmostEqual(bump.duration_at(3.0), 0.4, places=9)
        self.assertAlmostEqual(bump.duration_at(6.0), 0.2, places=9)

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            SpeedBump(height_m=0.0)
        with self.assertRaises(ValueError):
            SpeedBump(width_m=-1.0)
        with self.assertRaises(ValueError):
            SpeedBump(kind="triangle")
        with self.assertRaises(ValueError):
            SpeedBump(kind="trapezoid", ramp_ratio=0.6)
        with self.assertRaises(ValueError):
            SpeedBump().duration_at(0.0)

    def test_supported_kinds_are_declared(self):
        self.assertEqual(KINDS, ("half_sine", "trapezoid"))


if __name__ == "__main__":
    unittest.main()
