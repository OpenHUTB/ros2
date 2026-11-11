"""肌肉-肌腱单元的本构、串联平衡与二分法测试。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.muscle_tendon import (  # noqa: E402
    MuscleTendon,
    MuscleTendonParams,
    activation_step,
)


class TestConstitutiveCurves(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = MuscleTendonParams()
        cls.m = MuscleTendon(cls.p)

    def test_force_length_peak_at_optimum(self):
        self.assertAlmostEqual(self.m.force_length(1.0), 1.0, places=12)
        self.assertLess(self.m.force_length(0.9), 1.0)
        self.assertLess(self.m.force_length(1.1), 1.0)

    def test_force_length_symmetry(self):
        for dl in (0.05, 0.1, 0.2, 0.3):
            self.assertAlmostEqual(
                self.m.force_length(1.0 - dl),
                self.m.force_length(1.0 + dl),
                places=12,
            )

    def test_force_length_decays_away_from_optimum(self):
        self.assertGreater(self.m.force_length(0.85), self.m.force_length(0.7))
        self.assertGreater(self.m.force_length(0.95), self.m.force_length(0.85))

    def test_passive_force_zero_below_and_at_optimum(self):
        self.assertEqual(self.m.passive_force_length(0.8), 0.0)
        self.assertEqual(self.m.passive_force_length(1.0), 0.0)

    def test_passive_force_increases_above_optimum(self):
        f12 = self.m.passive_force_length(1.2)
        f13 = self.m.passive_force_length(1.3)
        self.assertGreater(f12, 0.0)
        self.assertGreater(f13, f12)

    def test_force_velocity_isometric(self):
        self.assertAlmostEqual(self.m.force_velocity(0.0), 1.0, places=12)

    def test_force_velocity_shortening_below_one(self):
        self.assertLess(self.m.force_velocity(0.5), 1.0)
        self.assertGreater(self.m.force_velocity(0.5), 0.0)
        self.assertEqual(self.m.force_velocity(1.0), 0.0)

    def test_force_velocity_lengthening_above_one(self):
        self.assertGreater(self.m.force_velocity(-0.5), 1.0)
        self.assertAlmostEqual(self.m.force_velocity(-0.5), 1.2, places=12)


class TestTendon(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = MuscleTendonParams()
        cls.m = MuscleTendon(cls.p)

    def test_tendon_slack_has_no_force(self):
        self.assertEqual(self.m.tendon_force(self.p.l_slack), 0.0)
        self.assertEqual(self.m.tendon_force(0.04), 0.0)

    def test_tendon_is_linear_above_slack(self):
        expected = self.p.k_t * 0.01
        self.assertAlmostEqual(self.m.tendon_force(0.06), expected, places=9)

    def test_tendon_energy_matches_force_squared(self):
        eq = self.m.equilibrium(self.p.rest_mtu, 0.6)
        self.assertAlmostEqual(
            eq.tendon_energy,
            eq.f_t ** 2 / (2.0 * self.p.k_t),
            places=12,
        )

    def test_k_t_definition(self):
        self.assertAlmostEqual(self.p.k_t, 30000.0, places=9)


class TestSeriesEquilibrium(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p = MuscleTendonParams()
        cls.m = MuscleTendon(cls.p)
        cls.rigid = MuscleTendon(cls.p, tendon_stiffness=math.inf)

    def test_residual_below_tolerance(self):
        for a in (0.2, 0.5, 0.8, 1.0):
            eq = self.m.equilibrium(self.p.rest_mtu, a)
            self.assertTrue(eq.converged, msg=f"a={a}")
            self.assertLess(abs(eq.residual), 1e-6, msg=f"a={a}")

    def test_higher_activation_shortens_fiber(self):
        lengths = [
            self.m.equilibrium(self.p.rest_mtu, a).l_m
            for a in (0.2, 0.4, 0.6, 0.8, 1.0)
        ]
        for shorter, longer in zip(lengths[1:], lengths[:-1]):
            self.assertLess(shorter, longer)

    def test_elastic_force_is_below_rigid(self):
        for a in (0.3, 0.5, 0.8, 1.0):
            e = self.m.equilibrium(self.p.rest_mtu, a)
            r = self.rigid.equilibrium(self.p.rest_mtu, a)
            self.assertLess(e.f_t, r.f_t, msg=f"a={a}")

    def test_rigid_limit_fiber_length_and_energy(self):
        eq = self.rigid.equilibrium(self.p.rest_mtu, 1.0)
        self.assertAlmostEqual(eq.l_m, self.p.rest_mtu - self.p.l_slack, places=12)
        self.assertAlmostEqual(eq.l_t, self.p.l_slack, places=12)
        self.assertEqual(eq.tendon_energy, 0.0)
        self.assertEqual(eq.tendon_strain, 0.0)
        self.assertAlmostEqual(eq.f_t, eq.f_m, places=12)

    def test_zero_activation_gives_zero_force(self):
        eq = self.m.equilibrium(self.p.rest_mtu, 0.0)
        self.assertEqual(eq.f_t, 0.0)
        self.assertEqual(eq.f_m, 0.0)
        self.assertEqual(eq.tendon_energy, 0.0)

    def test_no_nan_over_operating_range(self):
        for length in (0.051, 0.08, 0.135, 0.17):
            for a in (0.0, 0.25, 0.5, 0.75, 1.0):
                eq = self.m.equilibrium(length, a)
                self.assertTrue(math.isfinite(eq.f_t), msg=(length, a))
                self.assertTrue(math.isfinite(eq.l_m), msg=(length, a))

    def test_solve_fiber_length_returns_finite(self):
        l_m = self.m.solve_fiber_length(
            self.p.rest_mtu, 0.75, tol=1e-12, max_iter=200
        )
        self.assertTrue(math.isfinite(l_m))
        self.assertGreater(l_m, 0.0)
        self.assertLess(l_m, self.p.rest_mtu)

    def test_invalid_inputs_raise(self):
        with self.assertRaises(ValueError):
            self.m.equilibrium(-0.1, 0.5)
        with self.assertRaises(ValueError):
            self.m.equilibrium(0.135, -0.1)
        with self.assertRaises(ValueError):
            self.m.equilibrium(0.135, 0.5, tendon_stiffness=0.0)


class TestActivationDynamics(unittest.TestCase):
    def test_rise_towards_command(self):
        a = activation_step(0.0, 1.0, 0.01, tau_act=0.01, tau_deact=0.04)
        self.assertGreater(a, 0.0)
        self.assertLess(a, 1.0)
        self.assertAlmostEqual(a, 1.0 - math.exp(-1.0), places=12)

    def test_rise_is_faster_than_fall(self):
        rise = activation_step(0.0, 1.0, 0.02, 0.01, 0.04)
        fall = activation_step(1.0, 0.0, 0.02, 0.01, 0.04)
        self.assertGreater(rise, 1.0 - fall)

    def test_zero_dt_does_nothing(self):
        self.assertEqual(activation_step(0.3, 1.0, 0.0), 0.3)


if __name__ == "__main__":
    unittest.main()
