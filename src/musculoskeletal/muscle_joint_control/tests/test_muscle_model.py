# -*- coding: utf-8 -*-
"""Hill 肌肉模型的单元测试。

测试重点：各条生理曲线的**定性特征**与**边界条件**，而不是具体拟合值。
这些特征在生理学上是确定的，任何实现都不该违反，例如：

* 最优长度处主动力最大
* 短于最优长度时没有被动力
* 离心收缩的力大于等长收缩
* 激活快于放松
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from muscle_model import (  # noqa: E402
    Muscle,
    MuscleParams,
    activation_dynamics,
    force_length_active,
    force_length_passive,
    force_velocity,
    muscle_force,
)


# ===========================================================================
class TestMuscleParams(unittest.TestCase):
    """参数校验。"""

    def test_default_values(self):
        p = MuscleParams()
        self.assertAlmostEqual(p.f_max, 100.0)
        self.assertAlmostEqual(p.l_opt, 0.10)
        self.assertAlmostEqual(p.v_max, 1.0)
        self.assertLess(p.tau_act, p.tau_deact)

    def test_reject_non_positive_fmax(self):
        for bad in (0.0, -1.0):
            with self.assertRaises(ValueError):
                MuscleParams(f_max=bad)

    def test_reject_non_positive_lopt(self):
        with self.assertRaises(ValueError):
            MuscleParams(l_opt=0.0)

    def test_reject_non_positive_vmax(self):
        with self.assertRaises(ValueError):
            MuscleParams(v_max=-2.0)

    def test_reject_non_positive_tau(self):
        with self.assertRaises(ValueError):
            MuscleParams(tau_act=0.0)
        with self.assertRaises(ValueError):
            MuscleParams(tau_deact=-0.1)

    def test_reject_tau_deact_not_greater(self):
        """放松不应快于激活。"""
        with self.assertRaises(ValueError):
            MuscleParams(tau_act=0.05, tau_deact=0.05)
        with self.assertRaises(ValueError):
            MuscleParams(tau_act=0.05, tau_deact=0.01)

    def test_repr_contains_key_fields(self):
        s = repr(MuscleParams())
        self.assertIn("f_max", s)
        self.assertIn("tau_act", s)


# ===========================================================================
class TestActivationDynamics(unittest.TestCase):
    """激活动力学。"""

    def test_rising_toward_unit_command(self):
        a0 = 0.0
        a1 = activation_dynamics(1.0, a0, 0.005, tau_act=0.01)
        self.assertGreater(a1, a0)
        self.assertLess(a1, 1.0)

    def test_falling_toward_zero_command(self):
        a1 = activation_dynamics(0.0, 1.0, 0.005, tau_deact=0.04)
        self.assertLess(a1, 1.0)
        self.assertGreater(a1, 0.0)

    def test_steady_state_equals_command(self):
        """指令不变时激活度应收敛到指令值。"""
        a = 0.0
        for _ in range(2000):
            a = activation_dynamics(0.7, a, 0.001)
        self.assertAlmostEqual(a, 0.7, places=6)

    def test_activation_faster_than_deactivation(self):
        """同样时长内，上升幅度应大于下降幅度（tau_act < tau_deact）。"""
        rise = activation_dynamics(1.0, 0.0, 0.01) - 0.0
        fall = 1.0 - activation_dynamics(0.0, 1.0, 0.01)
        self.assertGreater(rise, fall)

    def test_command_clipped_above_one(self):
        a = 0.5
        for _ in range(5000):
            a = activation_dynamics(5.0, a, 0.001)
        self.assertLessEqual(a, 1.0)
        self.assertAlmostEqual(a, 1.0, places=6)

    def test_command_clipped_below_zero(self):
        a = 0.5
        for _ in range(5000):
            a = activation_dynamics(-3.0, a, 0.001)
        self.assertGreaterEqual(a, 0.0)
        self.assertAlmostEqual(a, 0.0, places=6)

    def test_output_always_in_range(self):
        """任意输入下输出都不越界。"""
        for u in (-1.0, 0.0, 0.5, 1.0, 2.0):
            for a in (0.0, 0.3, 0.9, 1.0):
                out = activation_dynamics(u, a, 0.05)
                self.assertGreaterEqual(out, 0.0)
                self.assertLessEqual(out, 1.0)

    def test_non_positive_dt_rejected(self):
        """dt 必须为正（0 与负数都拒绝）。"""
        with self.assertRaises(ValueError):
            activation_dynamics(1.0, 0.4, 0.0)
        with self.assertRaises(ValueError):
            activation_dynamics(1.0, 0.0, -0.01)

    def test_invalid_tau_rejected(self):
        with self.assertRaises(ValueError):
            activation_dynamics(1.0, 0.0, 0.01, tau_act=0.0)
        with self.assertRaises(ValueError):
            activation_dynamics(1.0, 0.0, 0.01, tau_deact=-1.0)

    def test_monotonic_in_time(self):
        """持续给 1 指令，激活度单调不减。"""
        a = 0.0
        prev = a
        for _ in range(200):
            a = activation_dynamics(1.0, a, 0.001)
            self.assertGreaterEqual(a + 1e-12, prev)
            prev = a


# ===========================================================================
class TestForceLengthActive(unittest.TestCase):
    """主动力-长度关系。"""

    def test_peak_at_optimal_length(self):
        self.assertAlmostEqual(force_length_active(1.0), 1.0, places=12)

    def test_decreases_both_sides(self):
        self.assertLess(force_length_active(0.8), 1.0)
        self.assertLess(force_length_active(1.2), 1.0)

    def test_symmetric_about_optimal(self):
        for d in (0.05, 0.1, 0.2, 0.3):
            self.assertAlmostEqual(force_length_active(1.0 - d),
                                   force_length_active(1.0 + d), places=12)

    def test_monotonic_up_to_peak(self):
        l = 0.5
        prev = force_length_active(l)
        while l < 1.0:
            l += 0.01
            cur = force_length_active(l)
            self.assertGreaterEqual(cur + 1e-12, prev)
            prev = cur

    def test_monotonic_after_peak(self):
        l = 1.0
        prev = force_length_active(l)
        while l < 1.5:
            l += 0.01
            cur = force_length_active(l)
            self.assertLessEqual(cur, prev + 1e-12)
            prev = cur

    def test_always_positive_and_bounded(self):
        for l in (0.1, 0.5, 1.0, 1.5, 2.0):
            v = force_length_active(l)
            self.assertGreater(v, 0.0)
            self.assertLessEqual(v, 1.0)

    def test_narrower_width_gives_smaller_force_off_peak(self):
        wide = force_length_active(0.7, width=0.6)
        narrow = force_length_active(0.7, width=0.3)
        self.assertGreater(wide, narrow)

    def test_invalid_width_rejected(self):
        with self.assertRaises(ValueError):
            force_length_active(1.0, width=0.0)


# ===========================================================================
class TestForceLengthPassive(unittest.TestCase):
    """被动力-长度关系。"""

    def test_zero_at_optimal_length(self):
        self.assertAlmostEqual(force_length_passive(1.0), 0.0, places=12)

    def test_zero_below_optimal_length(self):
        for l in (0.5, 0.8, 0.95, 0.999):
            self.assertAlmostEqual(force_length_passive(l), 0.0, places=12)

    def test_increases_with_length(self):
        prev = force_length_passive(1.05)
        l = 1.05
        while l < 1.55:
            l += 0.05
            cur = force_length_passive(l)
            self.assertGreater(cur, prev)
            prev = cur

    def test_reaches_one_at_lmax(self):
        self.assertAlmostEqual(force_length_passive(1.6, l_max=1.6), 1.0, places=9)

    def test_exceeds_one_beyond_lmax(self):
        self.assertGreater(force_length_passive(1.8, l_max=1.6), 1.0)

    def test_larger_k_is_more_convex(self):
        """曲线被 l_max 归一化后，k 越大中段越低（起始更平缓、末端更陡）。"""
        slow = force_length_passive(1.3, k=2.0)
        fast = force_length_passive(1.3, k=8.0)
        self.assertLess(fast, slow)

    def test_endpoints_fixed_regardless_of_k(self):
        """无论 k 取多少，l=1 处为 0、l=l_max 处为 1。"""
        for k in (1.0, 3.0, 5.0, 10.0):
            self.assertAlmostEqual(force_length_passive(1.0, k=k), 0.0, places=12)
            self.assertAlmostEqual(force_length_passive(1.6, k=k, l_max=1.6),
                                   1.0, places=9)

    def test_invalid_k_rejected(self):
        with self.assertRaises(ValueError):
            force_length_passive(1.2, k=0.0)

    def test_invalid_lmax_rejected(self):
        with self.assertRaises(ValueError):
            force_length_passive(1.2, l_max=1.0)


# ===========================================================================
class TestForceVelocity(unittest.TestCase):
    """力-速度关系。"""

    def test_isometric_is_one(self):
        self.assertAlmostEqual(force_velocity(0.0), 1.0, places=9)

    def test_zero_force_at_max_shortening_speed(self):
        self.assertAlmostEqual(force_velocity(1.0), 0.0, places=9)

    def test_no_negative_force_beyond_max_speed(self):
        for v in (1.0, 1.5, 3.0):
            self.assertGreaterEqual(force_velocity(v), 0.0)

    def test_eccentric_stronger_than_isometric(self):
        self.assertGreater(force_velocity(-0.5), 1.0)

    def test_eccentric_bounded_by_ecc_max(self):
        for v in (-0.5, -1.0, -5.0, -100.0):
            self.assertLessEqual(force_velocity(v), 1.4 + 1e-9)

    def test_monotonic_decreasing(self):
        v = -1.0
        prev = force_velocity(v)
        while v < 1.0:
            v += 0.01
            cur = force_velocity(v)
            self.assertLessEqual(cur, prev + 1e-12)
            prev = cur

    def test_continuous_at_zero(self):
        eps = 1e-9
        self.assertAlmostEqual(force_velocity(-eps), force_velocity(0.0), places=6)

    def test_shortening_reduces_force(self):
        self.assertLess(force_velocity(0.2), 1.0)
        self.assertLess(force_velocity(0.5), force_velocity(0.2))


# ===========================================================================
class TestMuscleForce(unittest.TestCase):
    """总肌肉力。"""

    def setUp(self):
        self.p = MuscleParams(f_max=100.0, l_opt=0.10, v_max=1.0)

    def test_zero_activation_isometric_gives_zero(self):
        """未激活且未拉长时，肌肉不产生力。"""
        self.assertAlmostEqual(
            muscle_force(0.0, 1.0, 0.0, self.p), 0.0, places=9)

    def test_full_activation_optimal_length_isometric_equals_fmax(self):
        self.assertAlmostEqual(
            muscle_force(1.0, 1.0, 0.0, self.p), 100.0, places=9)

    def test_force_scales_linearly_with_activation(self):
        f_half = muscle_force(0.5, 1.0, 0.0, self.p)
        f_full = muscle_force(1.0, 1.0, 0.0, self.p)
        self.assertAlmostEqual(f_half * 2.0, f_full, places=9)

    def test_activation_clipped(self):
        """激活度越界时按边界处理，不放大。"""
        self.assertAlmostEqual(
            muscle_force(3.0, 1.0, 0.0, self.p),
            muscle_force(1.0, 1.0, 0.0, self.p), places=9)
        self.assertAlmostEqual(
            muscle_force(-2.0, 1.0, 0.0, self.p),
            muscle_force(0.0, 1.0, 0.0, self.p), places=9)

    def test_passive_force_when_overstretched_even_without_activation(self):
        """拉长到远超最优长度时，即使不激活也有被动力。"""
        f = muscle_force(0.0, 1.5, 0.0, self.p)
        self.assertGreater(f, 0.0)

    def test_shortening_reduces_force(self):
        f_iso = muscle_force(1.0, 1.0, 0.0, self.p)
        f_short = muscle_force(1.0, 1.0, 0.5, self.p)
        self.assertLess(f_short, f_iso)

    def test_lengthening_increases_force(self):
        f_iso = muscle_force(1.0, 1.0, 0.0, self.p)
        f_ecc = muscle_force(1.0, 1.0, -0.5, self.p)
        self.assertGreater(f_ecc, f_iso)

    def test_default_params_used_when_none(self):
        self.assertAlmostEqual(
            muscle_force(1.0, 1.0, 0.0, None),
            muscle_force(1.0, 1.0, 0.0, MuscleParams()), places=9)

    def test_scales_with_fmax(self):
        p2 = MuscleParams(f_max=250.0, l_opt=0.10, v_max=1.0)
        self.assertAlmostEqual(
            muscle_force(1.0, 1.0, 0.0, p2),
            2.5 * muscle_force(1.0, 1.0, 0.0, self.p), places=6)


# ===========================================================================
class TestMuscleClass(unittest.TestCase):
    """带状态的 Muscle 封装。"""

    def test_initial_activation(self):
        m = Muscle(activation=0.3)
        self.assertAlmostEqual(m.activation, 0.3)

    def test_initial_activation_clipped(self):
        self.assertAlmostEqual(Muscle(activation=5.0).activation, 1.0)
        self.assertAlmostEqual(Muscle(activation=-1.0).activation, 0.0)

    def test_update_advances_activation(self):
        m = Muscle()
        m.update(1.0, 0.005)
        self.assertGreater(m.activation, 0.0)

    def test_force_uses_current_activation(self):
        m = Muscle(activation=1.0)
        self.assertAlmostEqual(m.force(1.0, 0.0), m.params.f_max, places=9)

    def test_reset(self):
        m = Muscle(activation=1.0)
        m.reset(0.0)
        self.assertAlmostEqual(m.activation, 0.0)

    def test_converges_to_command(self):
        m = Muscle()
        for _ in range(3000):
            m.update(0.6, 0.001)
        self.assertAlmostEqual(m.activation, 0.6, places=5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
