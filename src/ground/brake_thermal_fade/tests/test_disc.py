"""制动盘热模型与摩擦衰退曲线的解析校验。"""

import math
import unittest

import numpy as np

from brake_thermal_fade.disc import (
    BrakeDisc,
    BrakeDiscSpec,
    FrictionFadeCurve,
)


class FrictionCurveTest(unittest.TestCase):
    def setUp(self):
        self.curve = FrictionFadeCurve()

    def test_plateau_below_fade_and_floor_above(self):
        self.assertAlmostEqual(float(self.curve.mu(25.0)), 0.40, places=9)
        self.assertAlmostEqual(float(self.curve.mu(250.0)), 0.40, places=9)
        self.assertAlmostEqual(float(self.curve.mu(650.0)), 0.22, places=9)
        self.assertAlmostEqual(float(self.curve.mu(900.0)), 0.22, places=9)

    def test_linear_in_between(self):
        midway = 0.5 * (self.curve.fade_start_c + self.curve.fade_end_c)
        expected = 0.5 * (self.curve.mu_cold + self.curve.mu_hot)
        self.assertAlmostEqual(float(self.curve.mu(midway)), expected, places=9)

    def test_relative_is_mu_over_cold(self):
        self.assertAlmostEqual(float(self.curve.relative(25.0)), 1.0, places=12)
        self.assertAlmostEqual(float(self.curve.relative(650.0)), 0.22 / 0.40, places=12)

    def test_monotonic_non_increasing(self):
        temperatures = np.linspace(0.0, 900.0, 200)
        mu = self.curve.mu(temperatures)
        self.assertTrue(np.all(np.diff(mu) <= 1e-12))

    def test_invalid_curve_is_rejected(self):
        with self.assertRaises(ValueError):
            FrictionFadeCurve(mu_cold=0.30, mu_hot=0.40).validate()
        with self.assertRaises(ValueError):
            FrictionFadeCurve(fade_start_c=600.0, fade_end_c=300.0).validate()


class DiscThermalTest(unittest.TestCase):
    def setUp(self):
        # 关掉辐射，用线性散热与解析解对比
        self.spec = BrakeDiscSpec(emissivity=0.0)
        self.disc = BrakeDisc(spec=self.spec)

    def test_heat_capacity(self):
        self.assertAlmostEqual(self.disc.spec.heat_capacity_j_per_k, 7.0 * 500.0)

    def test_no_cooling_at_ambient(self):
        self.assertAlmostEqual(self.disc.cooling_power_w(self.spec.ambient_c), 0.0)

    def test_cooldown_matches_analytic_exponential(self):
        start = self.spec.ambient_c + 300.0
        temperature = start
        dt = 0.5
        for _ in range(int(120.0 / dt)):
            temperature = self.disc.step(temperature, 0.0, dt)
        tau = self.spec.heat_capacity_j_per_k / self.spec.cooling_w_per_k
        expected = self.spec.ambient_c + (start - self.spec.ambient_c) * math.exp(-120.0 / tau)
        self.assertAlmostEqual(temperature, expected, delta=0.2)

    def test_adiabatic_heating_matches_energy_balance(self):
        disc = BrakeDisc(
            spec=BrakeDiscSpec(emissivity=0.0, cooling_w_per_k=0.0),
        )
        temperature = disc.spec.ambient_c
        dt = 0.5
        power = 3000.0
        duration = 100.0
        for _ in range(int(duration / dt)):
            temperature = disc.step(temperature, power, dt)
        expected = disc.spec.ambient_c + power * duration / disc.spec.heat_capacity_j_per_k
        self.assertAlmostEqual(temperature, expected, delta=0.05)

    def test_radiation_adds_cooling_at_high_temperature(self):
        with_radiation = BrakeDisc()
        without = BrakeDisc(spec=BrakeDiscSpec(emissivity=0.0))
        self.assertGreater(
            with_radiation.cooling_power_w(600.0), without.cooling_power_w(600.0)
        )

    def test_equilibrium_temperature_matches_analytic(self):
        power = 3000.0
        equilibrium = self.disc.equilibrium_temperature_c(power)
        expected = self.spec.ambient_c + power / self.spec.cooling_w_per_k
        self.assertAlmostEqual(equilibrium, expected, delta=0.05)

    def test_equilibrium_is_none_when_power_exceeds_capacity(self):
        huge = self.disc.cooling_power_w(self.spec.max_temperature_c) * 2.0
        self.assertIsNone(self.disc.equilibrium_temperature_c(huge))

    def test_time_to_reach_is_ordered_and_bounded(self):
        power = 4000.0
        # 关掉辐射时平衡温度 = 环境 + 4000/15 ≈ 291.7 °C，所以 100/250 °C 可达
        early = self.disc.time_to_reach_c(power, 100.0)
        late = self.disc.time_to_reach_c(power, 250.0)
        self.assertIsNotNone(early)
        self.assertIsNotNone(late)
        self.assertLess(early, late)
        self.assertEqual(self.disc.time_to_reach_c(power, 10.0), 0.0)
        # 高于平衡温度的目标永远达不到
        self.assertIsNone(self.disc.time_to_reach_c(power, 300.0))
        self.assertIsNone(self.disc.time_to_reach_c(100.0, 800.0))

    def test_temperature_is_clamped_to_physical_range(self):
        hot = self.disc.step(self.spec.max_temperature_c, 1.0e6, 1.0)
        self.assertLessEqual(hot, self.spec.max_temperature_c)
        cold = self.disc.step(0.0, 0.0, 1.0)
        self.assertGreaterEqual(cold, self.spec.ambient_c)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            BrakeDiscSpec(disc_mass_kg=0.0).validate()
        with self.assertRaises(ValueError):
            BrakeDiscSpec(emissivity=1.5).validate()
        with self.assertRaises(ValueError):
            self.disc.step(100.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
