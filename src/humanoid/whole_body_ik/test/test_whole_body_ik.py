# -*- coding: utf-8 -*-
"""分层任务优先级全身 IK 的单元测试。

覆盖三块：

* 零空间投影矩阵的性质（J·N ≈ 0、幂等、递推后归零）
* 四种任务的误差 / 雅可比维数与边界行为
* 求解器：收敛性、任务优先级不被破坏、限位规避、参数校验

和 ``test_ik.py`` 一样，断言的都尽量是数学性质，而不是"跑通就行"。
"""

from __future__ import annotations

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "scripts"))

from ik_solver import solve_position_ik  # noqa: E402
from kinematics import Joint, make_humanoid_arm, make_planar_2r  # noqa: E402
from linalg import (  # noqa: E402
    identity,
    mat_mul,
    mat_vec,
    norm,
)
from manipulability import joint_limit_cost  # noqa: E402
from whole_body_ik import (  # noqa: E402
    LimitAvoidanceTask,
    PoseTask,
    PositionTask,
    PostureTask,
    solve_whole_body,
)

from whole_body_ik import _projector_update  # noqa: E402


def joint_margin(chain, q, index):
    """单关节归一化限位裕度（1 = 中点，0 = 贴边）。"""
    joint = chain.joints[index]
    if joint.limits is None:
        return 1.0
    lo, hi = joint.limits
    mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
    return 1.0 - abs(q[index] - mid) / half


def min_margin(chain, q):
    return min(joint_margin(chain, q, i) for i in range(chain.dof))


def tip_error(chain, q, target):
    p, _r = chain.fk(q)
    return norm([target[i] - p[i] for i in range(3)])


# ---------------------------------------------------------------------------
class TestProjector(unittest.TestCase):
    """零空间投影矩阵 N = I − J⁺J 的代数性质。"""

    def setUp(self):
        self.arm = make_humanoid_arm()
        self.q = [0.2, -0.3, 0.1, 1.0, 0.0, 0.2, 0.0]
        self.j = self.arm.position_jacobian(self.q)

    def test_task_direction_is_killed(self):
        n = _projector_update(identity(self.arm.dof), self.j, 1e-8)
        for col in range(self.arm.dof):
            v = [n[row][col] for row in range(self.arm.dof)]
            self.assertLess(norm(mat_vec(self.j, v)), 1e-5)

    def test_nontrivial_null_space(self):
        """7 自由度、3 维任务 → 零空间应该是 4 维，投影不能把一切都抹掉。"""
        n = _projector_update(identity(self.arm.dof), self.j, 1e-8)
        kept = 0
        for col in range(self.arm.dof):
            v = [n[row][col] for row in range(self.arm.dof)]
            if norm(v) > 0.5:
                kept += 1
        self.assertGreaterEqual(kept, 3)

    def test_idempotent(self):
        n = _projector_update(identity(self.arm.dof), self.j, 1e-8)
        nn = mat_mul(n, n)
        for i in range(self.arm.dof):
            for k in range(self.arm.dof):
                self.assertAlmostEqual(nn[i][k], n[i][k], places=5)

    def test_identity_jacobian_zeroes_projector(self):
        """关节空间任务（J = I）会占满所有自由度，之后零空间为 0。"""
        n = _projector_update(identity(self.arm.dof), identity(self.arm.dof), 1e-8)
        for row in n:
            for value in row:
                self.assertLess(abs(value), 1e-4)


# ---------------------------------------------------------------------------
class TestTasks(unittest.TestCase):
    def setUp(self):
        self.arm = make_humanoid_arm()
        self.q = [0.2, -0.3, 0.1, 1.0, 0.0, 0.2, 0.0]

    def test_position_task_dims(self):
        p, _r = self.arm.fk(self.q)
        task = PositionTask([p[0] + 0.1, p[1], p[2]])
        self.assertEqual(len(task.error(self.arm, self.q)), 3)
        self.assertEqual(len(task.jacobian(self.arm, self.q)), 3)
        self.assertEqual(len(task.jacobian(self.arm, self.q)[0]), self.arm.dof)

    def test_position_task_error_direction(self):
        p, _r = self.arm.fk(self.q)
        target = [p[0] + 0.1, p[1], p[2]]
        e = PositionTask(target).error(self.arm, self.q)
        self.assertAlmostEqual(e[0], 0.1, places=9)

    def test_position_task_rejects_bad_target(self):
        with self.assertRaises(ValueError):
            PositionTask([0.0, 0.0])

    def test_pose_task_is_six_dimensional(self):
        p, r = self.arm.fk(self.q)
        task = PoseTask(p, r)
        self.assertEqual(len(task.error(self.arm, self.q)), 6)
        self.assertEqual(len(task.jacobian(self.arm, self.q)), 6)

    def test_pose_task_rotation_weight(self):
        """姿态权重只影响后三行，位置部分原样保留。"""
        p, r = self.arm.fk(self.q)
        j1 = PoseTask(p, r, rotation_weight=0.3).jacobian(self.arm, self.q)
        j2 = PoseTask(p, r, rotation_weight=0.9).jacobian(self.arm, self.q)
        self.assertEqual(j1[:3], j2[:3])
        for a, b in zip(j1[3:], j2[3:]):
            for x, y in zip(a, b):
                self.assertAlmostEqual(3.0 * x, y, places=9)

    def test_pose_task_rejects_bad_rotation(self):
        with self.assertRaises(ValueError):
            PoseTask([0, 0, 0], [[1, 0], [0, 1]])

    def test_posture_task_is_joint_space(self):
        task = PostureTask([0.0] * self.arm.dof, weight=0.5)
        self.assertTrue(task.joint_space)
        self.assertEqual(len(task.error(self.arm, self.q)), self.arm.dof)
        j = task.jacobian(self.arm, self.q)
        for i in range(self.arm.dof):
            self.assertAlmostEqual(j[i][i], 1.0)

    def test_posture_task_rejects_wrong_dof(self):
        task = PostureTask([0.0] * 3)
        with self.assertRaises(ValueError):
            task.error(self.arm, self.q)

    def test_limit_task_silent_far_from_limits(self):
        q = [0.0, 0.0, 0.0, 1.3, 0.0, 0.0, 0.0]     # 全在限位中点附近
        task = LimitAvoidanceTask(margin=0.1)
        self.assertLess(norm(task.error(self.arm, q)), 1e-12)

    def test_limit_task_active_near_limit(self):
        q = [0.0, 0.0, 0.0, 2.55, 0.0, 0.0, 0.0]     # 肘部贴近上限 2.6
        task = LimitAvoidanceTask(margin=0.2, gain=1.0)
        e = task.error(self.arm, q)
        self.assertGreater(norm(e), 1e-3)
        self.assertLess(e[3], 0.0)                   # 应该往回退（负方向）

    def test_limit_task_clips_step(self):
        q = [0.0, 0.0, 0.0, 2.599, 0.0, 0.0, 0.0]
        task = LimitAvoidanceTask(margin=0.3, gain=5.0, max_step=0.05)
        self.assertLessEqual(norm(task.error(self.arm, q)), 0.05 + 1e-12)


# ---------------------------------------------------------------------------
class TestSolverBasics(unittest.TestCase):
    def setUp(self):
        self.arm = make_planar_2r(1.0, 1.0)
        self.target = [1.0, 0.6, 0.0]

    def test_converges_on_reachable_target(self):
        res = solve_whole_body(self.arm, [PositionTask(self.target)],
                               q0=[0.3, 0.5], tol=1e-6, max_iter=200)
        self.assertTrue(res.success)
        self.assertLess(res.error, 1e-6)
        self.assertLess(tip_error(self.arm, res.q, self.target), 1e-6)

    def test_result_bookkeeping(self):
        res = solve_whole_body(self.arm, [PositionTask(self.target)],
                               q0=[0.3, 0.5], tol=1e-6, max_iter=200)
        self.assertEqual(len(res.task_errors), 1)
        self.assertEqual(len(res.history), res.iterations + 1)
        self.assertIn("收敛", res.reason)

    def test_deterministic(self):
        a = solve_whole_body(self.arm, [PositionTask(self.target)],
                             q0=[0.3, 0.5], tol=1e-8, max_iter=200)
        b = solve_whole_body(self.arm, [PositionTask(self.target)],
                             q0=[0.3, 0.5], tol=1e-8, max_iter=200)
        for x, y in zip(a.q, b.q):
            self.assertAlmostEqual(x, y, places=12)

    def test_empty_tasks_rejected(self):
        with self.assertRaises(ValueError):
            solve_whole_body(self.arm, [])

    def test_bad_tolerance_rejected(self):
        with self.assertRaises(ValueError):
            solve_whole_body(self.arm, [PositionTask(self.target)], tol=0.0)

    def test_bad_q0_rejected(self):
        with self.assertRaises(ValueError):
            solve_whole_body(self.arm, [PositionTask(self.target)], q0=[0.0])

    def test_bad_step_size_rejected(self):
        with self.assertRaises(ValueError):
            solve_whole_body(self.arm, [PositionTask(self.target)], step_size=-1.0)

    def test_bad_step_tol_rejected(self):
        with self.assertRaises(ValueError):
            solve_whole_body(self.arm, [PositionTask(self.target)], step_tol=0.0)

    def test_unreachable_target_fails_gracefully(self):
        res = solve_whole_body(self.arm, [PositionTask([5.0, 0.0, 0.0])],
                               q0=[0.3, 0.5], tol=1e-6, max_iter=60)
        self.assertFalse(res.success)
        self.assertGreater(res.error, 1e-3)


# ---------------------------------------------------------------------------
class TestPriority(unittest.TestCase):
    """核心卖点：次级任务只在零空间里动，不破坏主任务。"""

    def setUp(self):
        self.arm = make_humanoid_arm()
        self.q0 = [0.3, -0.4, 0.2, 1.0, 0.1, 0.2, 0.0]
        p, _r = self.arm.fk(self.q0)
        self.target = [p[0] + 0.08, p[1] - 0.06, p[2] + 0.05]
        self.q_ref = [0.0, -0.3, 0.1, 1.2, 0.0, 0.2, 0.0]

    def test_primary_accuracy_preserved(self):
        single = solve_position_ik(self.arm, self.target, q0=self.q0,
                                   tol=1e-6, max_iter=300)
        whole = solve_whole_body(
            self.arm, [PositionTask(self.target), PostureTask(self.q_ref, 0.8)],
            q0=self.q0, tol=1e-6, max_iter=400)
        self.assertTrue(whole.success)
        self.assertLess(tip_error(self.arm, whole.q, self.target), 1e-6)
        # 和单任务解的末端位置一致（差别在微米以下）
        p_single, _ = self.arm.fk(single.q)
        p_whole, _ = self.arm.fk(whole.q)
        self.assertLess(norm([a - b for a, b in zip(p_single, p_whole)]), 1e-5)

    def test_posture_moves_towards_reference(self):
        single = solve_position_ik(self.arm, self.target, q0=self.q0,
                                   tol=1e-6, max_iter=300)
        whole = solve_whole_body(
            self.arm, [PositionTask(self.target), PostureTask(self.q_ref, 0.8)],
            q0=self.q0, tol=1e-6, max_iter=400)
        d_single = norm([a - b for a, b in zip(single.q, self.q_ref)])
        d_whole = norm([a - b for a, b in zip(whole.q, self.q_ref)])
        self.assertLess(d_whole, d_single - 0.05)

    def test_priority_order_matters(self):
        """把姿态任务放前面，主任务就该被牺牲——证明顺序真的生效。"""
        swapped = solve_whole_body(
            self.arm, [PostureTask(self.q_ref, 0.8), PositionTask(self.target)],
            q0=self.q0, tol=1e-6, max_iter=400)
        normal = solve_whole_body(
            self.arm, [PositionTask(self.target), PostureTask(self.q_ref, 0.8)],
            q0=self.q0, tol=1e-6, max_iter=400)
        self.assertGreater(tip_error(self.arm, swapped.q, self.target),
                           tip_error(self.arm, normal.q, self.target))

    def test_limit_avoidance_improves_margin(self):
        q0 = [0.1, -0.2, 0.05, 2.45, 0.0, 0.1, 0.0]
        p, _r = self.arm.fk(q0)
        target = [p[0] + 0.04, p[1] + 0.02, p[2] + 0.03]

        single = solve_position_ik(self.arm, target, q0=q0, tol=1e-6, max_iter=300)
        whole = solve_whole_body(
            self.arm,
            [PositionTask(target),
             LimitAvoidanceTask(margin=0.25, gain=3.0, max_step=0.08)],
            q0=q0, tol=1e-6, max_iter=400)

        self.assertTrue(whole.success)
        self.assertLess(tip_error(self.arm, whole.q, target), 1e-6)
        self.assertGreater(min_margin(self.arm, whole.q),
                           min_margin(self.arm, single.q) + 0.1)

    def test_limit_cost_drops(self):
        q0 = [0.1, -0.2, 0.05, 2.45, 0.0, 0.1, 0.0]
        p, _r = self.arm.fk(q0)
        target = [p[0] + 0.04, p[1] + 0.02, p[2] + 0.03]
        whole = solve_whole_body(
            self.arm,
            [PositionTask(target),
             LimitAvoidanceTask(margin=0.25, gain=3.0, max_step=0.08)],
            q0=q0, tol=1e-6, max_iter=400)
        cost, _ = joint_limit_cost(whole.q, self.arm.joints, margin=0.25)
        self.assertLess(cost, 1e-6)

    def test_whole_body_respects_joint_limits(self):
        q0 = [0.1, -0.2, 0.05, 2.45, 0.0, 0.1, 0.0]
        p, _r = self.arm.fk(q0)
        target = [p[0] + 0.3, p[1], p[2]]
        res = solve_whole_body(self.arm, [PositionTask(target)], q0=q0,
                               tol=1e-6, max_iter=100)
        self.assertTrue(self.arm.all_within_limits(res.q))


# ---------------------------------------------------------------------------
class TestPlanarChain(unittest.TestCase):
    """2R 臂（无冗余）也要能跑：没有零空间时次级任务自然不起作用。"""

    def test_two_dof_two_dimensional_task(self):
        arm = make_planar_2r(1.0, 1.0)
        target = [1.2, 0.3, 0.0]
        res = solve_whole_body(arm, [PositionTask(target)], q0=[0.2, 1.0],
                               tol=1e-6, max_iter=200)
        self.assertTrue(res.success)

    def test_no_null_space_for_posture(self):
        arm = make_planar_2r(1.0, 1.0)
        target = [1.2, 0.3, 0.0]
        q_ref = [0.0, 2.0]
        res = solve_whole_body(arm, [PositionTask(target), PostureTask(q_ref, 1.0)],
                               q0=[0.2, 1.0], tol=1e-6, max_iter=200)
        self.assertTrue(res.success)
        # 2 自由度做 3 维任务其实只有 2 个独立约束，仍可能有零空间；
        # 但至少不能破坏主任务
        self.assertLess(tip_error(arm, res.q, target), 1e-6)


if __name__ == "__main__":
    unittest.main()
