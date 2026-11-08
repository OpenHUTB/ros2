"""核心单元测试：肌肉模型、分配、QP。"""

import unittest

import numpy as np

from mujoco_muscle_leg_gait.core import (
    HillMuscle, Leg, allocate_pair, solve_joint_qp, R_HIP, R_KNEE, F_MAX,
)


class TestHillMuscle(unittest.TestCase):
    def test_fl_peak_at_optimal_length(self):
        self.assertAlmostEqual(HillMuscle.fl(1.0), 1.0, places=6)
        self.assertLess(HillMuscle.fl(0.8), 1.0)
        self.assertLess(HillMuscle.fl(1.2), 1.0)

    def test_passive_force_zero_below_optimal(self):
        for ln in (0.6, 0.9, 1.0):
            self.assertEqual(HillMuscle.fp(ln), 0.0)
        self.assertGreater(HillMuscle.fp(1.3), 0.0)

    def test_force_velocity_centrifugal_exceeds_isometric(self):
        self.assertAlmostEqual(HillMuscle.fv(0.0), 1.0, places=6)
        self.assertGreater(HillMuscle.fv(-0.5), 1.0)   # 离心更强
        self.assertLess(HillMuscle.fv(0.5), 1.0)       # 向心更弱

    def test_activation_faster_than_relaxation(self):
        m1, m2 = HillMuscle(0.3), HillMuscle(0.3)
        m1.update_activation(1.0, 0.01)     # 激活
        m2.a = 1.0
        m2.update_activation(0.0, 0.01)     # 放松
        self.assertGreater(m1.a, 1.0 - m2.a)

    def test_force_nonnegative(self):
        muscle = HillMuscle(0.3)
        muscle.a = 0.5
        for ln in np.linspace(0.5, 1.5, 11):
            self.assertGreaterEqual(muscle.force(0.3 * ln, 0.0), 0.0)


class TestAllocation(unittest.TestCase):
    def test_positive_torque_uses_first_muscle(self):
        a = allocate_pair(+1.0, R_HIP, F_MAX)
        self.assertGreater(a[0], a[1])

    def test_negative_torque_uses_second_muscle(self):
        a = allocate_pair(-1.0, R_HIP, F_MAX)
        self.assertGreater(a[1], a[0])

    def test_saturates_at_one(self):
        a = allocate_pair(1e6, R_HIP, F_MAX)
        self.assertAlmostEqual(a[0], 1.0)

    def test_knee_muscle_order(self):
        """膝肌序是 [ext, flex]，正力矩（屈膝）必须落在 flex（索引 3）。"""
        leg = Leg()
        u = leg.activations_for(np.array([0.0, +2.0]))
        self.assertGreater(u[3], u[2])
        u = leg.activations_for(np.array([0.0, -2.0]))
        self.assertGreater(u[2], u[3])


class TestQP(unittest.TestCase):
    def setUp(self):
        self.leg = Leg()
        self.A = self.leg.arms.T.copy() * F_MAX   # 力矩矩阵 (N·m per activation)

    def test_zero_torque_gives_zero_activation(self):
        a = solve_joint_qp(self.A, np.zeros(2))
        self.assertTrue(np.allclose(a, 0.0))

    def test_solution_matches_requested_torque(self):
        tau = np.array([1.0, 0.5])
        a = solve_joint_qp(self.A, tau)
        self.assertTrue(np.allclose(self.A @ a, tau, atol=1e-9))
        self.assertTrue(np.all(a >= -1e-12) and np.all(a <= 1.0 + 1e-12))

    def test_prefers_small_activation_norm(self):
        tau = np.array([0.0, 0.8])           # 只需屈膝
        a = solve_joint_qp(self.A, tau)
        self.assertGreater(a[3], 0.0)
        self.assertAlmostEqual(a.sum(), a[3], places=9)


if __name__ == "__main__":
    unittest.main()

