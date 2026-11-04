# -*- coding: utf-8 -*-
"""逆运动学与可操作性的单元测试。

覆盖四块：

* 线性代数底座（旋转、解方程、特征分解）
* 正运动学与雅可比（和解析解、数值差分对照）
* 逆解（收敛性、奇异点行为、零空间）
* 可操作性（和闭式解对照、椭球、限位代价）

断言的都是可独立验证的数学性质，不是"跑通就行"。
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

from ik_solver import (  # noqa: E402
    adaptive_damping,
    damped_step,
    default_initial_guess,
    null_space_project,
    pseudoinverse_step,
    rotation_error,
    smallest_singular_value,
    solve_position_ik,
    solve_pose_ik,
)
from kinematics import (  # noqa: E402
    Chain,
    Joint,
    make_humanoid_arm,
    make_planar_2r,
    planar_2r_fk,
    planar_2r_ik,
)
from linalg import (  # noqa: E402
    compose,
    cross,
    det,
    dot,
    identity,
    inverse_rigid,
    mat3_mul,
    mat3_vec,
    mat_mul,
    mat_vec,
    norm,
    normalize,
    rodrigues,
    rot_x,
    rot_y,
    rot_z,
    solve,
    symmetric_eigen,
    transpose,
)
from manipulability import (  # noqa: E402
    condition_number,
    is_singular,
    joint_limit_cost,
    manipulability,
    manipulability_ellipsoid,
    singular_values,
    singularity_distance,
)


def _max_abs_diff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


# ===========================================================================
class TestBasicLinalg(unittest.TestCase):
    def test_identity(self):
        m = identity(3)
        self.assertEqual(m[0], [1.0, 0.0, 0.0])
        self.assertEqual(m[2], [0.0, 0.0, 1.0])

    def test_mat_mul(self):
        a = [[1.0, 2.0], [3.0, 4.0]]
        b = [[5.0, 6.0], [7.0, 8.0]]
        self.assertEqual(mat_mul(a, b), [[19.0, 22.0], [43.0, 50.0]])

    def test_mat_mul_rectangular(self):
        a = [[1.0, 2.0, 3.0]]          # 1x3
        b = [[1.0], [1.0], [1.0]]      # 3x1
        self.assertEqual(mat_mul(a, b), [[6.0]])

    def test_mat_vec(self):
        a = [[2.0, 0.0], [0.0, 3.0]]
        self.assertEqual(mat_vec(a, [1.0, 2.0]), [2.0, 6.0])

    def test_transpose(self):
        self.assertEqual(transpose([[1, 2, 3], [4, 5, 6]]),
                         [[1, 4], [2, 5], [3, 6]])

    def test_dot_and_cross(self):
        self.assertAlmostEqual(dot([1, 2, 3], [4, 5, 6]), 32.0)
        self.assertEqual(cross([1, 0, 0], [0, 1, 0]), [0.0, 0.0, 1.0])

    def test_cross_is_anticommutative(self):
        a, b = [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]
        c1 = cross(a, b)
        c2 = cross(b, a)
        self.assertLess(_max_abs_diff(c1, [-x for x in c2]), 1e-12)

    def test_norm_and_normalize(self):
        self.assertAlmostEqual(norm([3.0, 4.0]), 5.0)
        n = normalize([0.0, 0.0, 7.0])
        self.assertEqual(n, [0.0, 0.0, 1.0])

    def test_normalize_zero_raises(self):
        with self.assertRaises(ValueError):
            normalize([0.0, 0.0, 0.0])


# ===========================================================================
class TestRotation(unittest.TestCase):
    def test_rot_z_90(self):
        r = rot_z(math.pi / 2)
        v = mat3_vec(r, [1.0, 0.0, 0.0])
        self.assertLess(_max_abs_diff(v, [0.0, 1.0, 0.0]), 1e-12)

    def test_rot_x_90(self):
        r = rot_x(math.pi / 2)
        v = mat3_vec(r, [0.0, 1.0, 0.0])
        self.assertLess(_max_abs_diff(v, [0.0, 0.0, 1.0]), 1e-12)

    def test_rot_y_90(self):
        r = rot_y(math.pi / 2)
        v = mat3_vec(r, [0.0, 0.0, 1.0])
        self.assertLess(_max_abs_diff(v, [1.0, 0.0, 0.0]), 1e-12)

    def test_rotation_orthonormal(self):
        r = rodrigues([1.0, 2.0, 3.0], 0.7)
        rt = transpose(r)
        prod = mat3_mul(r, rt)
        for i in range(3):
            for j in range(3):
                expect = 1.0 if i == j else 0.0
                self.assertAlmostEqual(prod[i][j], expect, places=12)

    def test_rotation_determinant_is_one(self):
        r = rodrigues([0.3, -0.7, 0.5], 1.1)
        self.assertAlmostEqual(det(r), 1.0, places=12)

    def test_rodrigues_zero_angle_is_identity(self):
        r = rodrigues([0.0, 0.0, 1.0], 0.0)
        self.assertLess(_max_abs_diff([x for row in r for x in row],
                                      [x for row in identity(3) for x in row]),
                        1e-12)

    def test_rodrigues_axis_is_invariant(self):
        axis = normalize([1.0, 1.0, 0.0])
        r = rodrigues(axis, 0.9)
        v = mat3_vec(r, axis)
        self.assertLess(_max_abs_diff(v, axis), 1e-12)


# ===========================================================================
class TestSolveAndDet(unittest.TestCase):
    def test_solve_simple(self):
        x = solve([[3.0, 1.0], [1.0, 2.0]], [9.0, 8.0])
        self.assertLess(_max_abs_diff(x, [2.0, 3.0]), 1e-12)

    def test_solve_identity(self):
        x = solve(identity(3), [1.0, 2.0, 3.0])
        self.assertLess(_max_abs_diff(x, [1.0, 2.0, 3.0]), 1e-12)

    def test_solve_singular_raises(self):
        with self.assertRaises(ValueError):
            solve([[1.0, 2.0], [2.0, 4.0]], [1.0, 2.0])

    def test_solve_dimension_mismatch_raises(self):
        with self.assertRaises(ValueError):
            solve([[1.0, 2.0], [3.0, 4.0]], [1.0])

    def test_det_2x2(self):
        self.assertAlmostEqual(det([[3.0, 1.0], [1.0, 2.0]]), 5.0, places=12)

    def test_det_identity(self):
        self.assertAlmostEqual(det(identity(4)), 1.0, places=12)

    def test_det_singular_is_zero(self):
        self.assertAlmostEqual(det([[1.0, 2.0], [2.0, 4.0]]), 0.0, places=12)

    def test_det_triangular(self):
        self.assertAlmostEqual(det([[2.0, 9.0], [0.0, 3.0]]), 6.0, places=12)


# ===========================================================================
class TestSymmetricEigen(unittest.TestCase):
    def test_known_matrix(self):
        a = [[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 5.0]]
        vals, _vecs = symmetric_eigen(a)
        self.assertLess(_max_abs_diff(vals, [5.0, 3.0, 1.0]), 1e-9)

    def test_descending_order(self):
        a = [[1.0, 0.0], [0.0, 4.0]]
        vals, _ = symmetric_eigen(a)
        self.assertGreaterEqual(vals[0], vals[1])

    def test_eigenvectors_orthonormal(self):
        a = [[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 1.0]]
        _vals, vecs = symmetric_eigen(a)
        for i in range(3):
            self.assertAlmostEqual(norm(vecs[i]), 1.0, places=10)
            for j in range(i + 1, 3):
                self.assertAlmostEqual(dot(vecs[i], vecs[j]), 0.0, places=9)

    def test_eigen_residual(self):
        a = [[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 1.0]]
        vals, vecs = symmetric_eigen(a)
        for k in range(3):
            av = mat_vec(a, vecs[k])
            lv = [vals[k] * x for x in vecs[k]]
            self.assertLess(_max_abs_diff(av, lv), 1e-9)

    def test_diagonal_matrix(self):
        a = [[3.0, 0.0], [0.0, -1.0]]
        vals, _ = symmetric_eigen(a)
        self.assertLess(_max_abs_diff(vals, [3.0, -1.0]), 1e-12)


# ===========================================================================
class TestPoseHelpers(unittest.TestCase):
    def test_compose_and_inverse(self):
        r = rot_z(0.6)
        p = [1.0, 2.0, 3.0]
        t = compose(r, p)
        inv = inverse_rigid(t)
        prod = mat_mul(t, inv)
        for i in range(4):
            for j in range(4):
                expect = 1.0 if i == j else 0.0
                self.assertAlmostEqual(prod[i][j], expect, places=11)


# ===========================================================================
class TestJoint(unittest.TestCase):
    def test_axis_normalized(self):
        j = Joint("a", axis=(0.0, 0.0, 5.0))
        self.assertAlmostEqual(j.axis[2], 1.0)

    def test_zero_axis_raises(self):
        with self.assertRaises(ValueError):
            Joint("a", axis=(0.0, 0.0, 0.0))

    def test_clamp(self):
        j = Joint("a", limits=(-1.0, 1.0))
        self.assertAlmostEqual(j.clamp(2.0), 1.0)
        self.assertAlmostEqual(j.clamp(-2.0), -1.0)
        self.assertAlmostEqual(j.clamp(0.5), 0.5)

    def test_clamp_without_limits(self):
        j = Joint("a")
        self.assertAlmostEqual(j.clamp(99.0), 99.0)

    def test_within_limits(self):
        j = Joint("a", limits=(0.0, 1.0))
        self.assertTrue(j.within_limits(0.5))
        self.assertTrue(j.within_limits(0.0))
        self.assertFalse(j.within_limits(1.5))

    def test_bad_limits_raise(self):
        with self.assertRaises(ValueError):
            Joint("a", limits=(1.0, -1.0))


# ===========================================================================
class TestChain(unittest.TestCase):
    def setUp(self):
        self.arm = make_planar_2r(1.0, 1.0)

    def test_dof_and_reach(self):
        self.assertEqual(self.arm.dof, 2)
        self.assertAlmostEqual(self.arm.reach(), 2.0, places=12)

    def test_empty_chain_raises(self):
        with self.assertRaises(ValueError):
            Chain([])

    def test_wrong_q_length_raises(self):
        with self.assertRaises(ValueError):
            self.arm.fk([0.0])

    def test_fk_matches_analytic(self):
        for q in ([0.0, 0.0], [0.3, -0.5], [1.2, 0.8], [-2.0, 1.5]):
            p_generic, _ = self.arm.fk(q)
            p_analytic = planar_2r_fk(q, 1.0, 1.0)
            self.assertLess(_max_abs_diff(p_generic, p_analytic), 1e-12)

    def test_fk_straight_out(self):
        p, _ = self.arm.fk([0.0, 0.0])
        self.assertAlmostEqual(p[0], 2.0, places=12)
        self.assertAlmostEqual(p[1], 0.0, places=12)

    def test_jacobian_matches_numeric(self):
        q = [math.pi / 2, 0.4]
        j = self.arm.position_jacobian(q)
        h = 1e-7
        for c in range(2):
            qp = list(q)
            qp[c] += h
            qm = list(q)
            qm[c] -= h
            pp = planar_2r_fk(qp, 1.0, 1.0)
            pm = planar_2r_fk(qm, 1.0, 1.0)
            for r in range(3):
                num = (pp[r] - pm[r]) / (2.0 * h)
                self.assertAlmostEqual(j[r][c], num, places=6)

    def test_full_jacobian_shape(self):
        j = self.arm.jacobian([0.2, 0.3])
        self.assertEqual(len(j), 6)
        self.assertEqual(len(j[0]), 2)

    def test_clamp_all(self):
        j1 = Joint("a", axis=(0, 0, 1), limits=(-0.5, 0.5))
        j2 = Joint("b", axis=(0, 0, 1), origin=(1, 0, 0), limits=(-0.5, 0.5))
        c = Chain([j1, j2], tool=(1, 0, 0))
        q = c.clamp_all([2.0, -2.0])
        self.assertAlmostEqual(q[0], 0.5)
        self.assertAlmostEqual(q[1], -0.5)

    def test_humanoid_arm_dof(self):
        arm = make_humanoid_arm()
        self.assertEqual(arm.dof, 7)
        self.assertGreater(arm.reach(), 0.5)

    def test_humanoid_arm_jacobian_shape(self):
        arm = make_humanoid_arm()
        j = arm.jacobian(default_initial_guess(arm))
        self.assertEqual(len(j), 6)
        self.assertEqual(len(j[0]), 7)


# ===========================================================================
class TestPlanarAnalytic(unittest.TestCase):
    def test_ik_roundtrip(self):
        for tgt in ((1.2, 0.6), (0.0, 1.5), (-0.8, -1.0)):
            q = planar_2r_ik(tgt[0], tgt[1], 1.0, 1.0)
            back = planar_2r_fk(q, 1.0, 1.0)
            self.assertAlmostEqual(back[0], tgt[0], places=10)
            self.assertAlmostEqual(back[1], tgt[1], places=10)

    def test_ik_out_of_reach_raises(self):
        with self.assertRaises(ValueError):
            planar_2r_ik(5.0, 0.0, 1.0, 1.0)

    def test_elbow_up_vs_down(self):
        q_up = planar_2r_ik(1.2, 0.6, 1.0, 1.0, elbow_up=True)
        q_dn = planar_2r_ik(1.2, 0.6, 1.0, 1.0, elbow_up=False)
        self.assertAlmostEqual(q_up[1], -q_dn[1], places=12)

    def test_make_planar_invalid_lengths(self):
        with self.assertRaises(ValueError):
            make_planar_2r(0.0, 1.0)


# ===========================================================================
class TestManipulability(unittest.TestCase):
    def setUp(self):
        self.arm = make_planar_2r(1.0, 1.0)

    def test_2r_closed_form(self):
        """2R 的可操作性有闭式解 w = |l1 l2 sin q1|。"""
        for q1 in (0.0, 0.3, 0.5, 1.0, 1.57, 2.2):
            j = self.arm.position_jacobian([0.4, q1])[:2]
            w = manipulability(j)
            self.assertAlmostEqual(w, abs(math.sin(q1)), places=10)

    def test_max_at_right_angle(self):
        j = self.arm.position_jacobian([0.4, math.pi / 2])[:2]
        self.assertAlmostEqual(manipulability(j), 1.0, places=10)

    def test_zero_when_fully_extended(self):
        for q1 in (0.0, math.pi):
            j = self.arm.position_jacobian([0.4, q1])[:2]
            self.assertAlmostEqual(manipulability(j), 0.0, places=10)

    def test_singular_flags(self):
        j_ext = self.arm.position_jacobian([0.0, 0.0])[:2]
        j_bent = self.arm.position_jacobian([0.4, 1.0])[:2]
        self.assertTrue(is_singular(j_ext))
        self.assertFalse(is_singular(j_bent))

    def test_condition_number_at_singularity_is_inf(self):
        j = self.arm.position_jacobian([0.0, 0.0])[:2]
        self.assertEqual(condition_number(j), float("inf"))

    def test_condition_number_decreases_toward_right_angle(self):
        conds = []
        for q1 in (0.2, 0.6, 1.0, math.pi / 2):
            j = self.arm.position_jacobian([0.4, q1])[:2]
            conds.append(condition_number(j))
        for a, b in zip(conds, conds[1:]):
            self.assertGreater(a, b)

    def test_singular_values_descending(self):
        arm = make_humanoid_arm()
        j = arm.position_jacobian(default_initial_guess(arm))
        sv = singular_values(j)
        for a, b in zip(sv, sv[1:]):
            self.assertGreaterEqual(a, b)

    def test_singular_values_match_manipulability(self):
        """w 应该等于奇异值之积。"""
        arm = make_humanoid_arm()
        j = arm.position_jacobian(default_initial_guess(arm))
        sv = singular_values(j)
        prod = 1.0
        for s in sv:
            prod *= s
        self.assertAlmostEqual(manipulability(j), prod, places=9)

    def test_ellipsoid_axes_are_singular_values(self):
        arm = make_humanoid_arm()
        j = arm.position_jacobian(default_initial_guess(arm))
        ell = manipulability_ellipsoid(j)
        sv = singular_values(j)
        self.assertLess(_max_abs_diff(ell["axes"], sv), 1e-9)

    def test_ellipsoid_positive_volume(self):
        arm = make_humanoid_arm()
        j = arm.position_jacobian(default_initial_guess(arm))
        ell = manipulability_ellipsoid(j)
        self.assertGreater(ell["volume"], 0.0)

    def test_singularity_distance_of_planar(self):
        j = self.arm.position_jacobian([0.0, 0.0])[:2]
        self.assertAlmostEqual(singularity_distance(j), 0.0, places=10)


# ===========================================================================
class TestJointLimitCost(unittest.TestCase):
    def test_zero_cost_in_middle(self):
        j = Joint("a", limits=(-1.0, 1.0))
        cost, grad = joint_limit_cost([0.0], [j])
        self.assertAlmostEqual(cost, 0.0)
        self.assertAlmostEqual(grad[0], 0.0)

    def test_cost_grows_near_limit(self):
        j = Joint("a", limits=(-1.0, 1.0))
        c_mid, _ = joint_limit_cost([0.0], [j])
        c_near, _ = joint_limit_cost([0.95], [j])
        self.assertGreater(c_near, c_mid)

    def test_no_limits_no_cost(self):
        j = Joint("a")
        cost, grad = joint_limit_cost([5.0], [j])
        self.assertAlmostEqual(cost, 0.0)
        self.assertAlmostEqual(grad[0], 0.0)


# ===========================================================================
class TestIKSolver(unittest.TestCase):
    def setUp(self):
        self.arm = make_planar_2r(1.0, 1.0)
        self.h7 = make_humanoid_arm()

    def test_2r_converges(self):
        for tgt in ([1.2, 0.6, 0.0], [0.0, 1.5, 0.0], [1.5, -0.4, 0.0]):
            res = solve_position_ik(self.arm, tgt, tol=1e-6)
            self.assertTrue(res.success, "目标 %s 未收敛" % (tgt,))
            self.assertLess(res.error, 1e-6)

    def test_2r_matches_analytic_position(self):
        tgt = [1.2, 0.6, 0.0]
        res = solve_position_ik(self.arm, tgt, tol=1e-8)
        p, _ = self.arm.fk(res.q)
        self.assertLess(_max_abs_diff(p, tgt), 1e-6)

    def test_7dof_converges(self):
        q_true = [0.3, -0.4, 0.2, 0.9, 0.1, 0.3, -0.2]
        target, _ = self.h7.fk(q_true)
        res = solve_position_ik(self.h7, target, tol=1e-6)
        self.assertTrue(res.success)
        self.assertLess(res.error, 1e-6)

    def test_default_guess_is_bent(self):
        """默认初值应该让肘关节离开 0（避开奇异）。"""
        g = default_initial_guess(self.h7)
        self.assertGreater(abs(g[3]), 0.5)

    def test_default_guess_without_limits_is_zero(self):
        g = default_initial_guess(self.arm)
        self.assertEqual(g, [0.0, 0.0])

    def test_unreachable_target_fails(self):
        res = solve_position_ik(self.arm, [10.0, 0.0, 0.0], max_iter=50)
        self.assertFalse(res.success)
        self.assertGreater(res.error, 1.0)

    def test_history_is_recorded(self):
        res = solve_position_ik(self.arm, [1.2, 0.6, 0.0])
        self.assertGreater(len(res.history), 0)
        # 误差总体下降
        self.assertLess(res.history[-1], res.history[0])

    def test_joint_limits_respected(self):
        res = solve_position_ik(self.h7, [0.12, 0.0, -0.45])
        self.assertTrue(self.h7.all_within_limits(res.q, tol=1e-6))

    def test_bad_target_dimension_raises(self):
        with self.assertRaises(ValueError):
            solve_position_ik(self.arm, [1.0, 2.0])

    def test_bad_tol_raises(self):
        with self.assertRaises(ValueError):
            solve_position_ik(self.arm, [1.0, 0.0, 0.0], tol=0.0)

    def test_bad_max_iter_raises(self):
        with self.assertRaises(ValueError):
            solve_position_ik(self.arm, [1.0, 0.0, 0.0], max_iter=0)

    def test_wrong_q0_dimension_raises(self):
        with self.assertRaises(ValueError):
            solve_position_ik(self.arm, [1.0, 0.0, 0.0], q0=[0.0])

    def test_circle_tracking_accuracy(self):
        """沿圆跟踪，每个点都应精确收敛。"""
        center, radius = [0.0, 0.0, -0.45], 0.12
        q = None
        for i in range(12):
            ang = 2.0 * math.pi * i / 12.0
            tgt = [center[0] + radius * math.cos(ang),
                   center[1] + radius * math.sin(ang),
                   center[2]]
            res = solve_position_ik(self.h7, tgt, q0=q, tol=1e-6)
            self.assertTrue(res.success, "第 %d 点未收敛" % i)
            q = res.q


# ===========================================================================
class TestSingularityHandling(unittest.TestCase):
    def setUp(self):
        self.arm = make_planar_2r(1.0, 1.0)

    def test_pseudoinverse_raises_at_singularity(self):
        j = self.arm.position_jacobian([0.0, 0.0])[:2]
        with self.assertRaises(ValueError):
            pseudoinverse_step(j, [0.1, 0.0])

    def test_damped_works_at_singularity(self):
        j = self.arm.position_jacobian([0.0, 0.0])[:2]
        dq = damped_step(j, [0.0, 0.1], 0.05)
        self.assertEqual(len(dq), 2)
        self.assertLess(norm(dq), 1.0)

    def test_damped_negative_lambda_raises(self):
        j = self.arm.position_jacobian([0.4, 1.0])[:2]
        with self.assertRaises(ValueError):
            damped_step(j, [0.1, 0.0], -0.1)

    def test_pseudoinverse_dq_grows_near_singularity(self):
        """伪逆步长按 1/σ_min 增长。"""
        norms = []
        for q1 in (0.1, 0.02, 0.005):
            j = self.arm.position_jacobian([0.0, q1])[:2]
            norms.append(norm(pseudoinverse_step(j, [0.01, 0.0])))
        for a, b in zip(norms, norms[1:]):
            self.assertGreater(b, a * 2.0)

    def test_dls_dq_bounded_near_singularity(self):
        """DLS 步长保持有界，不随 σ_min 增大。"""
        norms = []
        for q1 in (0.1, 0.02, 0.005):
            j = self.arm.position_jacobian([0.0, q1])[:2]
            lam = adaptive_damping(j)
            norms.append(norm(damped_step(j, [0.01, 0.0], lam)))
        self.assertLess(max(norms), 1.0)
        # 越接近奇异，步长反而越小
        self.assertGreater(norms[0], norms[-1])

    def test_adaptive_lambda_increases_near_singularity(self):
        """只有 σ_min 掉到阈值以下时 λ 才开始增大。

        阈值默认 0.05，所以 q1=0.2（σ_min≈0.089）仍在"不阻尼"区间，
        从 q1=0.1 起才能真正看到 λ 随奇异程度上升。
        """
        lams = []
        for q1 in (0.1, 0.05, 0.03, 0.01):
            j = self.arm.position_jacobian([0.0, q1])[:2]
            lams.append(adaptive_damping(j))
        for a, b in zip(lams, lams[1:]):
            self.assertLess(a, b)

    def test_adaptive_lambda_flat_when_far_from_singularity(self):
        """离奇异很远时 λ 保持在下限，不无谓地牺牲精度。"""
        lam_far = adaptive_damping(self.arm.position_jacobian([0.4, 1.5])[:2])
        lam_min_default = 1e-4
        self.assertAlmostEqual(lam_far, lam_min_default, places=9)

    def test_adaptive_lambda_bounded(self):
        for q1 in (0.0, 0.001, 0.5, 1.5):
            j = self.arm.position_jacobian([0.0, q1])[:2]
            lam = adaptive_damping(j, lam_min=1e-4, lam_max=0.2)
            self.assertGreaterEqual(lam, 1e-4 - 1e-12)
            self.assertLessEqual(lam, 0.2 + 1e-12)

    def test_smallest_singular_value_zero_at_singular(self):
        j = self.arm.position_jacobian([0.0, 0.0])[:2]
        self.assertAlmostEqual(smallest_singular_value(j), 0.0, places=10)


# ===========================================================================
class TestNullSpace(unittest.TestCase):
    def test_projection_does_not_move_end_effector(self):
        arm = make_humanoid_arm()
        q = default_initial_guess(arm)
        j = arm.position_jacobian(q)
        z = [1.0, -0.5, 0.3, -0.2, 0.8, -0.6, 0.4]
        proj = null_space_project(j, z, lam=1e-8)
        moved = mat_vec(j, proj)
        self.assertLess(norm(moved), 1e-6)

    def test_projection_reduces_component(self):
        arm = make_humanoid_arm()
        j = arm.position_jacobian(default_initial_guess(arm))
        z = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        proj = null_space_project(j, z, lam=1e-8)
        self.assertLessEqual(norm(proj), norm(z) + 1e-9)


# ===========================================================================
class TestRotationError(unittest.TestCase):
    def test_identity_gives_zero(self):
        e = rotation_error(identity(3), identity(3))
        self.assertLess(norm(e), 1e-9)

    def test_pure_z_rotation(self):
        e = rotation_error(rot_z(0.5), identity(3))
        self.assertAlmostEqual(e[2], 0.5, places=9)
        self.assertAlmostEqual(e[0], 0.0, places=9)

    def test_magnitude_between_rotations(self):
        e = rotation_error(rot_z(0.8), rot_z(0.2))
        self.assertAlmostEqual(norm(e), 0.6, places=6)


# ===========================================================================
class TestPoseIK(unittest.TestCase):
    def test_reaches_pose_on_7dof(self):
        """7 自由度手臂能同时满足位置和姿态（6 维任务）。

        注意不能用 2R 平面臂测位姿逆解：它只能实现 3 维（平面内 2 个位置
        + 绕 z 的 1 个姿态），6 维误差里有一半根本不可达，算法再对也收敛不了。
        """
        arm = make_humanoid_arm()
        q_true = [0.2, -0.2, 0.1, 0.9, 0.0, 0.2, 0.0]
        p_true, r_true = arm.fk(q_true)
        res = solve_pose_ik(arm, p_true, r_true, tol=1e-5, max_iter=600)
        self.assertTrue(res.success)
        self.assertLess(res.error, 1e-4)

    def test_bad_target_rot_raises(self):
        arm = make_planar_2r(1.0, 1.0)
        with self.assertRaises(ValueError):
            solve_pose_ik(arm, [1.0, 0.0, 0.0], [[1.0, 0.0], [0.0, 1.0]])


if __name__ == "__main__":
    unittest.main(verbosity=2)
