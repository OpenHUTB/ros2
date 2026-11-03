"""纵向制动动力学：减速度、停车距离与单盘功率。"""

import math
import unittest

from brake_thermal_fade.disc import FrictionFadeCurve
from brake_thermal_fade.vehicle import (
    BrakingSpec,
    deceleration_mps2,
    per_disc_power_w,
    stop_energy_j,
    stopping_distance_m,
)


class BrakingTest(unittest.TestCase):
    def setUp(self):
        self.braking = BrakingSpec()
        self.curve = FrictionFadeCurve()

    def test_reference_deceleration_at_ambient(self):
        self.assertAlmostEqual(
            deceleration_mps2(25.0, self.braking, self.curve),
            self.braking.reference_decel_mps2,
            places=9,
        )

    def test_deceleration_drops_inside_fade_zone(self):
        cold = deceleration_mps2(25.0, self.braking, self.curve)
        warm = deceleration_mps2(450.0, self.braking, self.curve)
        hot = deceleration_mps2(700.0, self.braking, self.curve)
        self.assertGreater(cold, warm)
        self.assertGreater(warm, hot)
        self.assertAlmostEqual(hot, self.braking.reference_decel_mps2 * 0.22 / 0.40, places=9)

    def test_stopping_distance_matches_analytic(self):
        speed = 100.0 / 3.6
        distance = stopping_distance_m(speed, 25.0, self.braking, self.curve)
        expected = speed**2 / (2.0 * self.braking.reference_decel_mps2)
        self.assertAlmostEqual(distance, expected, places=9)
        self.assertAlmostEqual(distance, 48.225, places=2)

    def test_distance_grows_with_temperature(self):
        speed = 100.0 / 3.6
        distances = [
            stopping_distance_m(speed, temperature, self.braking, self.curve)
            for temperature in (25.0, 300.0, 500.0, 800.0)
        ]
        self.assertEqual(distances, sorted(distances))
        self.assertGreater(distances[-1], distances[0] * 1.5)

    def test_zero_speed_gives_zero_distance(self):
        self.assertAlmostEqual(
            stopping_distance_m(0.0, 400.0, self.braking, self.curve), 0.0
        )

    def test_stop_energy_is_kinetic_energy(self):
        speed = 20.0
        self.assertAlmostEqual(
            stop_energy_j(speed, self.braking),
            0.5 * self.braking.mass_kg * speed**2,
            places=6,
        )

    def test_per_disc_power_splits_between_four_brakes(self):
        speed = 60.0 / 3.6
        grade = 0.07
        power = per_disc_power_w(speed, grade, self.braking)
        total = self.braking.mass_kg * self.braking.gravity * grade * speed
        self.assertAlmostEqual(power * self.braking.brake_count, total, places=6)

    def test_flat_road_needs_no_braking_power(self):
        self.assertAlmostEqual(per_disc_power_w(15.0, 0.0, self.braking), 0.0)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            BrakingSpec(mass_kg=0.0).validate()
        with self.assertRaises(ValueError):
            BrakingSpec(reference_decel_mps2=0.0).validate()
        with self.assertRaises(ValueError):
            BrakingSpec(brake_count=0).validate()
        with self.assertRaises(ValueError):
            stopping_distance_m(-1.0, 25.0, self.braking, self.curve)
        with self.assertRaises(ValueError):
            per_disc_power_w(10.0, 1.5, self.braking)


if __name__ == "__main__":
    unittest.main()
