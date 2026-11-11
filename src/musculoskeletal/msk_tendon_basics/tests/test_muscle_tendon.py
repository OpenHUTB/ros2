# -*- coding: utf-8 -*-
"""肌肉-肌腱单元模型测试。"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from muscle_tendon import (  # noqa: E402
    TendonParams,
    activation_step,
    fiber_force,
    force_length_active,
    force_length_passive,
    force_velocity,
    mtu_state,
    solve_fiber_length,
    tendon_energy,
    tendon_force,
)


class ParamsTest(unittest.TestCase):
    def test_default_stiffness(self):
        p = TendonParams()
        self.assertAlmostEqual(p.tendon_stiffness, 30000.0, places=6)

    def test_invalid_values(self):
        for kwargs in ({"f_max": 0}, {"l_opt": 0}, {"v_max": -1},
                       {"l_slack": 0}, {"tendon_strain": 0}, {"width": 0},
                       {"tau_act": 0}, {"tau_deact": 0.01}):
            with self.assertRaises(ValueError):
                TendonParams(**kwargs)

    def test_zero_slack_never_divides_by_zero(self):
        p = TendonParams()
        p.l_slack = 0.0
        with self.assertRaises(ValueError):
            p.tendon_stiffness


class HillCurvesTest(unittest.TestCase):
    def test_active_peak_at_optimum(self):
        self.assertAlmostEqual(force_length_active(1.0), 1.0)
        self.assertLess(force_length_active(0.8), 1.0)
        self.assertLess(force_length_active(1.2), 1.0)
        self.assertAlmostEqual(force_length_active(0.8), force_length_active(1.2))

    def test_passive_only_when_stretched(self):
        self.assertEqual(force_length_passive(1.0), 0.0)
        self.assertEqual(force_length_passive(0.9), 0.0)
        self.assertGreater(force_length_passive(1.2), 0.0)

    def test_velocity_curve(self):
        self.assertAlmostEqual(force_velocity(0.0), 1.0)
        self.assertLess(force_velocity(0.5), 1.0)
        self.assertLess(force_velocity(1.0), 1e-6)
        self.assertGreater(force_velocity(-0.5), 1.0)
        self.assertLessEqual(force_velocity(-10.0), 1.4)

    def test_eccentric_branch_approaches_asymptote(self):
        # 不能把 -v 截断在 1，否则 v<=-1 后力固定 1.32，永远到不了 1.4
        f1 = force_velocity(-1.0)
        f5 = force_velocity(-5.0)
        self.assertGreater(f5, f1)
        self.assertGreater(f5, 1.35)
        self.assertLess(f5, 1.4)


class ActivationTest(unittest.TestCase):
    def test_activation_faster_than_relaxation(self):
        a_on = activation_step(1.0, 0.0, 0.01)
        a_off = activation_step(0.0, 1.0, 0.01)
        self.assertGreater(a_on, 1.0 - a_off)

    def test_clipped(self):
        self.assertEqual(activation_step(2.0, 0.0, 1.0), 1.0)
        self.assertEqual(activation_step(-1.0, 1.0, 1.0), 0.0)

    def test_invalid_dt(self):
        with self.assertRaises(ValueError):
            activation_step(1.0, 0.0, 0.0)


class TendonTest(unittest.TestCase):
    def test_slack(self):
        p = TendonParams()
        self.assertEqual(tendon_force(p.l_slack - 0.01, p), 0.0)
        self.assertEqual(tendon_force(p.l_slack, p), 0.0)

    def test_linear(self):
        p = TendonParams()
        f1 = tendon_force(p.l_slack + 0.001, p)
        f2 = tendon_force(p.l_slack + 0.002, p)
        self.assertAlmostEqual(f2, 2.0 * f1)

    def test_energy_formula(self):
        p = TendonParams()
        f = tendon_force(p.l_slack + 0.002, p)
        self.assertAlmostEqual(tendon_energy(f, p.tendon_stiffness),
                               f * f / (2.0 * p.tendon_stiffness))


class EquilibriumTest(unittest.TestCase):
    def setUp(self):
        self.p = TendonParams()

    def test_force_balance(self):
        for a in (0.0, 0.2, 0.5, 0.8, 1.0):
            for L in (0.130, 0.135, 0.140):
                l_m, force = solve_fiber_length(L, a, self.p)
                f_t = tendon_force(L - l_m, self.p)
                f_m = fiber_force(a, l_m, 0.0, self.p)
                self.assertAlmostEqual(f_t, f_m, delta=1e-5)
                self.assertAlmostEqual(force, f_t, delta=1e-5)

    def test_higher_activation_shortens_fiber(self):
        lengths = []
        for a in (0.0, 0.25, 0.5, 0.75, 1.0):
            l_m, _ = solve_fiber_length(0.135, a, self.p)
            lengths.append(l_m)
        for prev, cur in zip(lengths, lengths[1:]):
            self.assertGreater(prev, cur)

    def test_elastic_tendon_reduces_force(self):
        _, f_elastic = solve_fiber_length(0.135, 1.0, self.p,
                                          self.p.tendon_stiffness)
        _, f_rigid = solve_fiber_length(0.135, 1.0, self.p, 1e12)
        self.assertLess(f_elastic, f_rigid)

    def test_rigid_limit_fiber_length(self):
        l_m, _ = solve_fiber_length(0.135, 1.0, self.p, 1e12)
        self.assertAlmostEqual(l_m, 0.135 - self.p.l_slack, delta=1e-6)


class StateTest(unittest.TestCase):
    def test_state_fields_and_energy(self):
        p = TendonParams()
        s = mtu_state(0.135, 1.0, p)
        for key in ("l_m", "l_t", "force", "fiber_force", "tendon_force",
                    "fiber_length_norm", "tendon_strain", "energy"):
            self.assertIn(key, s)
        self.assertAlmostEqual(s["l_m"] + s["l_t"], 0.135, places=9)
        self.assertGreater(s["energy"], 0.0)
        self.assertAlmostEqual(s["tendon_strain"],
                               (s["l_t"] - p.l_slack) / p.l_slack)


if __name__ == "__main__":
    unittest.main()
