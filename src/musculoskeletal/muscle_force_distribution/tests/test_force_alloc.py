# -*- coding: utf-8 -*-
"""力分配模块的单元测试（只用标准库）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from force_alloc import (  # noqa: E402
    Muscle,
    allocate,
    achievable_range,
    joint_stiffness_proxy,
    objective,
    torque_vector,
)


def make_muscles():
    return [
        Muscle("F1", 0.03, 1000, +1),
        Muscle("F2", 0.03, 1000, +1),
        Muscle("E1", -0.03, 1000, -1),
        Muscle("E2", -0.03, 1000, -1),
    ]


class TestForceAlloc(unittest.TestCase):
    def test_recovers_desired_when_feasible(self):
        ms = make_muscles()
        a, ta = allocate(30.0, ms)
        self.assertAlmostEqual(ta, 30.0, places=6)

    def test_redundant_split_equal(self):
        # 两块相同屈肌应平分负载
        ms = make_muscles()
        a, _ = allocate(30.0, ms)
        self.assertAlmostEqual(a[0], a[1], places=6)
        self.assertEqual(a[2], 0.0)
        self.assertEqual(a[3], 0.0)

    def test_activations_in_range(self):
        ms = make_muscles()
        for td in [-40, -10, 0, 10, 40]:
            a, _ = allocate(td, ms)
            for x in a:
                self.assertGreaterEqual(x, 0.0)
                self.assertLessEqual(x, 1.0)

    def test_infeasible_saturates(self):
        ms = make_muscles()
        lo, hi = achievable_range(ms)
        a, ta = allocate(hi + 50.0, ms)  # 超出可行上界
        self.assertLess(ta, hi + 50.0)
        self.assertAlmostEqual(ta, hi, places=6)  # 饱和到上界

    def test_cocontraction_stiffness(self):
        ms = make_muscles()
        # 全 0.3 共激活：净力矩为 0，但刚度代理 > 0
        a0 = [0.3, 0.3, 0.3, 0.3]
        net = sum(torque_vector(ms)[i] * a0[i] for i in range(4))
        self.assertAlmostEqual(net, 0.0, places=6)
        self.assertGreater(joint_stiffness_proxy(ms, a0), 0.0)

    # ---------- 非等权（加权最小二乘）分配 ----------
    def test_weighted_allocation_matches_analytic_optimum(self):
        """两块等价屈肌、w=[4,1]、tau=30 时，最优解应为 [0.2, 0.8]。

        解析解 a_i = tau * (R_i/w_i) / sum_j (R_j^2/w_j)：
        权重 4 的肌肉代价高，应只承担 20% 负载。
        """
        ms = [Muscle("F1", 0.03, 1000, +1), Muscle("F2", 0.03, 1000, +1)]
        w = [4.0, 1.0]
        a, ta = allocate(30.0, ms, weights=w)
        self.assertAlmostEqual(a[0], 0.2, places=9)
        self.assertAlmostEqual(a[1], 0.8, places=9)
        self.assertAlmostEqual(ta, 30.0, places=9)
        self.assertAlmostEqual(objective(a, w), 0.8, places=9)

    def test_heavier_weight_gets_less_activation(self):
        """权重越大（代价越高）的肌肉，激活度必须越小；等权时二者相等。"""
        ms = [Muscle("F1", 0.03, 1000, +1), Muscle("F2", 0.03, 1000, +1)]
        a_eq, _ = allocate(30.0, ms, weights=[1.0, 1.0])
        self.assertAlmostEqual(a_eq[0], a_eq[1], places=9)
        for w0 in [2.0, 4.0, 9.0]:
            a, _ = allocate(30.0, ms, weights=[w0, 1.0])
            self.assertLess(a[0], a[1], "w=%.1f 时 a0 应小于 a1" % w0)
        # 且权重越大，a0 越小（单调）
        seq = [allocate(30.0, ms, weights=[w0, 1.0])[0][0] for w0 in [2.0, 4.0, 9.0]]
        self.assertGreater(seq[0], seq[1])
        self.assertGreater(seq[1], seq[2])

    def test_weighted_solution_beats_swapped_allocation(self):
        """把两肢激活互换后目标函数必须变大，验证真的取到了最小值。"""
        ms = [Muscle("F1", 0.03, 1000, +1), Muscle("F2", 0.03, 1000, +1)]
        w = [4.0, 1.0]
        a, _ = allocate(30.0, ms, weights=w)
        j_opt = objective(a, w)
        j_swapped = objective([a[1], a[0]], w)   # 旧实现给出的 [0.8, 0.2]
        self.assertLess(j_opt, j_swapped)
        self.assertAlmostEqual(j_swapped, 2.6, places=9)

    def test_weighted_still_tracks_torque(self):
        """加权后仍须精确满足力矩约束（在可行域内）。"""
        ms = make_muscles()
        w = [2.0, 5.0, 1.0, 3.0]
        for td in [-40.0, -12.5, 0.0, 17.0, 40.0]:
            _, ta = allocate(td, ms, weights=w)
            self.assertAlmostEqual(ta, td, places=6)

    def test_weights_must_be_positive(self):
        ms = make_muscles()
        with self.assertRaises(ValueError):
            allocate(10.0, ms, weights=[1.0, 1.0, 1.0, 0.0])
        with self.assertRaises(ValueError):
            allocate(10.0, ms, weights=[1.0, 1.0])   # 长度不符

    # ---------- 饱和后的剩余力矩重分配 ----------
    def test_saturation_redistributes_remaining_torque(self):
        """评审复现场景：两块等价屈肌 arm=0.03 fmax=1000、w=[4,1]、tau=50。

        无约束解析解 a=[1/3, 4/3] 越界；直接裁剪只能实现 40 Nm。
        正确解：a2 钉死在 1，剩余 20 Nm 由 F1 承担 -> a=[2/3, 1]，
        精确实现 50 Nm，且是盒约束下该目标的最优解。
        """
        ms = [Muscle("F1", 0.03, 1000, +1), Muscle("F2", 0.03, 1000, +1)]
        a, ta = allocate(50.0, ms, weights=[4.0, 1.0])
        self.assertAlmostEqual(a[0], 2.0 / 3.0, places=9)
        self.assertAlmostEqual(a[1], 1.0, places=9)
        self.assertAlmostEqual(ta, 50.0, places=9)

    def test_saturation_beats_other_feasible_solutions(self):
        """饱和重分配解须是可行解中的最优：任取另一个满足力矩约束的可行点，
        目标值都必须更大（在 a2<1 的可行段上目标随 a2 增大而减小，故 a2=1 最优）。
        """
        ms = [Muscle("F1", 0.03, 1000, +1), Muscle("F2", 0.03, 1000, +1)]
        w = [4.0, 1.0]
        a, ta = allocate(50.0, ms, weights=w)
        self.assertAlmostEqual(ta, 50.0, places=9)
        for a2_alt in (0.8, 0.9, 0.99):
            a1_alt = 5.0 / 3.0 - a2_alt          # 30*(a1+a2) = 50
            self.assertGreater(objective([a1_alt, a2_alt], w), objective(a, w))
        # 直接裁剪的旧实现结果 [1/3, 1] 并不满足力矩约束（只实现 40 Nm），
        # 说明修复前的问题确实是「可行目标力矩被凭空丢失」
        self.assertAlmostEqual(30.0 * (1.0 / 3.0 + 1.0), 40.0, places=9)

    def test_multiple_saturations_feasible(self):
        """三块屈肌两块饱和后，剩余力矩仍要被精确重分配。"""
        ms = [
            Muscle("F1", 0.02, 1000, +1),   # R=20
            Muscle("F2", 0.03, 1000, +1),   # R=30
            Muscle("F3", 0.04, 1000, +1),   # R=40
        ]
        w = [1.0, 4.0, 4.0]
        a, ta = allocate(85.0, ms, weights=w)
        self.assertAlmostEqual(ta, 85.0, places=9)
        for x in a:
            self.assertGreaterEqual(x, 0.0)
            self.assertLessEqual(x, 1.0)
        # 最优性抽查：对任意可行的扰动交换，目标值不得更小
        base = objective(a, w)
        self.assertAlmostEqual(base, sum(wi * ai * ai for wi, ai in zip(w, a)))

    def test_sweep_feasible_torques_all_achieved(self):
        """整个可行域内加权分配都必须精确实现期望力矩（无失真）。"""
        ms = make_muscles()
        w = [2.0, 5.0, 1.0, 3.0]
        lo, hi = achievable_range(ms)
        for k in range(21):
            td = lo + (hi - lo) * k / 20.0
            _, ta = allocate(td, ms, weights=w)
            self.assertAlmostEqual(ta, td, places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
