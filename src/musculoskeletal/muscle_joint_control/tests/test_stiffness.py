# -*- coding: utf-8 -*-
"""拮抗肌共激活与关节刚度的单元测试。

测试重点是**已被数值验证的物理规律**，尤其是共激活对刚度的影响
如何随工作点（上升段 / 最优长度 / 下降段）改变方向。
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))

from joint_model import JointParams  # noqa: E402
from stiffness import (  # noqa: E402
    disturbance_response,
    energy_cost,
    joint_stiffness,
    operating_segment,
    slope_balance,
    static_torque,
    stiffness_sweep,
    tradeoff_analysis,
)


def params_at(ratio, **kw):
    """构造 l_rest = ratio · l_opt 的参数组。"""
    return JointParams(rest_length=0.10 * ratio, l_opt=0.10, **kw)


# ===========================================================================
class TestStaticTorque(unittest.TestCase):
    def test_zero_at_neutral_with_equal_activation(self):
        """两侧同样激活时，θ=0 处力矩为零。"""
        p = params_at(1.0)
        self.assertAlmostEqual(static_torque(p, 0.0, 0.3, 0.3), 0.0, places=12)

    def test_torque_opposes_rotation(self):
        """稳定关节的力矩与偏转反号。"""
        p = params_at(1.0)
        for th in (-0.2, -0.1, 0.1, 0.2):
            tau = static_torque(p, th, 0.3, 0.3)
            self.assertLess(tau * th, 0.0)

    def test_antisymmetric_in_theta(self):
        """两侧激活相同时，对称参数下 τ(−θ) = −τ(θ)。

        注意：激活不对称时 τ(0) ≠ 0，反对称性不成立。
        """
        p = params_at(1.0)
        for th in (0.05, 0.15, 0.25):
            self.assertAlmostEqual(static_torque(p, -th, 0.3, 0.3),
                                   -static_torque(p, th, 0.3, 0.3), places=12)

    def test_asymmetric_activation_breaks_antisymmetry(self):
        p = params_at(1.0)
        self.assertNotAlmostEqual(static_torque(p, -0.15, 0.3, 0.2),
                                  -static_torque(p, 0.15, 0.3, 0.2), places=6)

    def test_asymmetric_activation_produces_torque_at_neutral(self):
        """只激活屈肌时，θ=0 处应产生正力矩。"""
        p = params_at(1.0)
        self.assertGreater(static_torque(p, 0.0, 0.8, 0.2), 0.0)
        self.assertLess(static_torque(p, 0.0, 0.2, 0.8), 0.0)

    def test_passive_only_restoring_torque(self):
        """零激活不等于零力矩。

        θ > 0 时伸肌被拉过最优长度，即使完全不激活也会产生被动力，
        从而给出负的恢复力矩。这正是被动力-长度关系的体现。
        """
        p = params_at(1.0)
        self.assertAlmostEqual(static_torque(p, 0.0, 0.0, 0.0), 0.0, places=12)
        self.assertLess(static_torque(p, 0.15, 0.0, 0.0), 0.0)
        self.assertGreater(static_torque(p, -0.15, 0.0, 0.0), 0.0)


# ===========================================================================
class TestJointStiffness(unittest.TestCase):
    def test_positive_for_stable_joint(self):
        """上升段与最优长度处刚度应为正（恢复力矩）。"""
        for ratio in (0.90, 0.95, 0.98, 1.00, 1.02):
            p = params_at(ratio)
            self.assertGreater(joint_stiffness(p, 0.0, 0.2, 0.2), 0.0,
                               msg=f"ratio={ratio}")

    def test_even_in_theta(self):
        """刚度关于 θ=0 对称（该点两侧曲线对称）。"""
        p = params_at(1.0)
        for th in (0.05, 0.1, 0.2):
            self.assertAlmostEqual(joint_stiffness(p, th, 0.3, 0.3),
                                   joint_stiffness(p, -th, 0.3, 0.3),
                                   places=6)

    def test_reject_non_positive_dtheta(self):
        p = params_at(1.0)
        with self.assertRaises(ValueError):
            joint_stiffness(p, 0.0, 0.2, 0.2, dtheta=0.0)
        with self.assertRaises(ValueError):
            joint_stiffness(p, 0.0, 0.2, 0.2, dtheta=-1e-6)

    def test_scales_with_moment_arm_squared(self):
        """由 k = r²·(∂F/∂l) 可知，刚度正比于力矩臂平方。"""
        p1 = JointParams(rest_length=0.095, l_opt=0.10, moment_arm=0.02)
        p2 = JointParams(rest_length=0.095, l_opt=0.10, moment_arm=0.04)
        k1 = joint_stiffness(p1, 0.0, 0.3, 0.3)
        k2 = joint_stiffness(p2, 0.0, 0.3, 0.3)
        self.assertAlmostEqual(k2 / k1, 4.0, places=1)


# ===========================================================================
class TestOperatingSegment(unittest.TestCase):
    def test_classification(self):
        self.assertEqual(operating_segment(params_at(0.95)), "上升段")
        self.assertEqual(operating_segment(params_at(1.00)), "最优长度")
        self.assertEqual(operating_segment(params_at(1.05)), "下降段")
        self.assertEqual(operating_segment(params_at(1.70)), "过度拉长")

    def test_near_optimal_tolerance(self):
        """l_rest 在最优长度附近的小偏差仍算"最优长度"。"""
        self.assertEqual(operating_segment(params_at(1.002)), "最优长度")
        self.assertEqual(operating_segment(params_at(0.998)), "最优长度")


# ===========================================================================
class TestSlopeBalance(unittest.TestCase):
    """机理分解：主动项 + 被动项 = 稳定性判据。"""

    def test_ascending_limb_active_slope_positive(self):
        b = slope_balance(params_at(0.95), a=0.5)
        self.assertGreater(b["active"], 0.0)
        self.assertEqual(b["passive"], 0.0)   # l_n < 1 无被动力
        self.assertTrue(b["stable"])

    def test_descending_limb_active_slope_negative(self):
        b = slope_balance(params_at(1.05), a=0.5)
        self.assertLess(b["active"], 0.0)
        self.assertGreater(b["passive"], 0.0)

    def test_descending_limb_becomes_unstable_at_high_activation(self):
        """下降段：激活足够高时主动负斜率压过被动正斜率 → 失稳。"""
        p = params_at(1.05)
        self.assertTrue(slope_balance(p, a=0.2)["stable"])
        self.assertFalse(slope_balance(p, a=1.0)["stable"])

    def test_optimal_length_active_slope_is_zero(self):
        b = slope_balance(params_at(1.00), a=0.8)
        self.assertAlmostEqual(b["active"], 0.0, places=9)

    def test_total_equals_sum(self):
        b = slope_balance(params_at(1.05), a=0.4)
        self.assertAlmostEqual(b["total"], b["active"] + b["passive"], places=12)


# ===========================================================================
class TestStiffnessSweep(unittest.TestCase):
    """共激活对刚度的效应方向随工作点改变 —— 本模块的核心结论。"""

    def test_ascending_limb_stiffness_increases_with_coactivation(self):
        p = params_at(0.95)
        rows = stiffness_sweep(p, [0.05, 0.2, 0.5, 0.9])
        ks = [k for _c, k in rows]
        for i in range(1, len(ks)):
            self.assertGreater(ks[i], ks[i - 1])

    def test_descending_limb_stiffness_decreases_with_coactivation(self):
        p = params_at(1.05)
        rows = stiffness_sweep(p, [0.05, 0.2, 0.5, 0.9])
        ks = [k for _c, k in rows]
        for i in range(1, len(ks)):
            self.assertLess(ks[i], ks[i - 1])

    def test_descending_limb_can_reach_negative_stiffness(self):
        """下降段高共激活时刚度变负 —— 这是模型的真实预测，不是错误。"""
        p = params_at(1.05)
        self.assertLess(stiffness_sweep(p, [0.9])[0][1], 0.0)

    def test_optimal_length_stiffness_independent_of_coactivation(self):
        p = params_at(1.00)
        rows = stiffness_sweep(p, [0.05, 0.3, 0.6, 0.9])
        ks = [k for _c, k in rows]
        self.assertAlmostEqual(max(ks) - min(ks), 0.0, places=9)

    def test_reject_empty_levels(self):
        with self.assertRaises(ValueError):
            stiffness_sweep(params_at(1.0), [])

    def test_reject_out_of_range_levels(self):
        with self.assertRaises(ValueError):
            stiffness_sweep(params_at(1.0), [-0.1])
        with self.assertRaises(ValueError):
            stiffness_sweep(params_at(1.0), [1.5])

    def test_returns_pairs_in_order(self):
        rows = stiffness_sweep(params_at(1.0), [0.4, 0.1, 0.2])
        self.assertEqual([c for c, _k in rows], [0.4, 0.1, 0.2])


# ===========================================================================
class TestDisturbanceResponse(unittest.TestCase):
    def test_reject_bad_arguments(self):
        with self.assertRaises(ValueError):
            disturbance_response(params_at(1.0), dt=0.0)
        with self.assertRaises(ValueError):
            disturbance_response(params_at(1.0), duration=0.0)

    def test_deflects_in_direction_of_external_torque(self):
        r = disturbance_response(params_at(1.0), co=0.2, tau_ext=0.05,
                                 duration=1.5, dt=0.001)
        self.assertGreater(r["theta_ss"], 0.0)

    def test_negative_torque_deflects_negative(self):
        r = disturbance_response(params_at(1.0), co=0.2, tau_ext=-0.05,
                                 duration=1.5, dt=0.001)
        self.assertLess(r["theta_ss"], 0.0)

    def test_higher_stiffness_means_smaller_deflection(self):
        """上升段上提高共激活会增大刚度，偏转应更小。"""
        p = params_at(0.95)
        low = disturbance_response(p, co=0.05, tau_ext=0.05, duration=1.5)
        high = disturbance_response(p, co=0.8, tau_ext=0.05, duration=1.5)
        self.assertLess(abs(high["theta_ss"]), abs(low["theta_ss"]))

    def test_equivalent_stiffness_finite_and_positive(self):
        r = disturbance_response(params_at(1.0), co=0.3, tau_ext=0.05,
                                 duration=1.5, dt=0.001)
        self.assertGreater(r["k_equivalent"], 0.0)
        self.assertNotEqual(r["k_equivalent"], float("inf"))

    def test_equivalent_stiffness_close_to_intrinsic(self):
        """小扰动下等效刚度应接近解析求得的关节刚度。"""
        p = params_at(0.95)
        k_intrinsic = joint_stiffness(p, 0.0, 0.3, 0.3)
        r = disturbance_response(p, co=0.3, tau_ext=0.01, duration=2.0)
        self.assertGreater(r["k_equivalent"], 0.0)
        # 量级一致即可（等效刚度还含阻尼的静态无贡献，但强直条件下应同阶）
        self.assertLess(abs(r["k_equivalent"] / k_intrinsic), 5.0)

    def test_with_controller_reduces_deflection(self):
        """叠加 PD 位置控制后，同样扰动下的偏转应更小。"""
        p = params_at(1.0)
        plain = disturbance_response(p, co=0.2, tau_ext=0.05, duration=1.5,
                                     use_controller=False)
        ctrl = disturbance_response(p, co=0.2, tau_ext=0.05, duration=1.5,
                                    use_controller=True)
        self.assertLess(abs(ctrl["theta_ss"]), abs(plain["theta_ss"]))

    def test_history_recorded(self):
        r = disturbance_response(params_at(1.0), co=0.2, tau_ext=0.05,
                                 duration=0.5, dt=0.001)
        self.assertEqual(len(r["history"]["t"]), 500)

    def test_max_deflection_at_least_final(self):
        r = disturbance_response(params_at(1.0), co=0.2, tau_ext=0.05,
                                 duration=1.5, dt=0.001)
        self.assertGreaterEqual(r["max_deflection"], abs(r["theta_ss"]) - 1e-9)


# ===========================================================================
class TestEnergyCost(unittest.TestCase):
    def test_zero_at_zero_coactivation(self):
        self.assertAlmostEqual(energy_cost(0.0, 1.0), 0.0)

    def test_linear_in_coactivation(self):
        self.assertAlmostEqual(energy_cost(0.4, 1.0), 2.0 * energy_cost(0.2, 1.0))

    def test_linear_in_duration(self):
        self.assertAlmostEqual(energy_cost(0.3, 2.0), 2.0 * energy_cost(0.3, 1.0))

    def test_reject_bad_arguments(self):
        with self.assertRaises(ValueError):
            energy_cost(1.5, 1.0)
        with self.assertRaises(ValueError):
            energy_cost(0.3, -1.0)


# ===========================================================================
class TestTradeoffAnalysis(unittest.TestCase):
    def test_returns_one_row_per_level(self):
        rows = tradeoff_analysis(params_at(0.95), [0.02, 0.15, 0.35],
                                 duration=1.0, dt=0.001)
        self.assertEqual(len(rows), 3)
        for key in ("co", "k", "ss_error", "rise_time", "energy"):
            self.assertIn(key, rows[0])

    def test_energy_increases_with_coactivation(self):
        rows = tradeoff_analysis(params_at(0.95), [0.02, 0.2, 0.5],
                                 duration=1.0, dt=0.001)
        for i in range(1, len(rows)):
            self.assertGreater(rows[i]["energy"], rows[i - 1]["energy"])

    def test_ascending_limb_stiffness_increases_but_accuracy_worsens(self):
        """上升段：共激活提高刚度，但跟踪误差变大 —— 这就是权衡。"""
        rows = tradeoff_analysis(params_at(0.95), [0.02, 0.5],
                                 duration=1.5, dt=0.001)
        self.assertGreater(rows[1]["k"], rows[0]["k"])
        self.assertGreater(abs(rows[1]["ss_error"]), abs(rows[0]["ss_error"]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
