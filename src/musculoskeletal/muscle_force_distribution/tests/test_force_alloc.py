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


if __name__ == "__main__":
    unittest.main(verbosity=2)
