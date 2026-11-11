"""全身逆运动学与可操作性测试。

运行：python -m pytest -q
"""

from __future__ import annotations

import numpy as np
import pytest

from scripts.ik_solver import solve_pose_ik, solve_position_ik, track_circle
from scripts.kinematics import (
    Joint,
    SerialChain,
    make_2r_arm,
    make_7dof_arm,
    reference_circle_pose,
)
from scripts.linalg import (
    condition_number,
    joint_limit_cost,
    nullspace_projector,
    orientation_error,
    rotation_matrix,
    rotation_x,
    rotation_y,
    rotation_z,
    so3_log,
)
from scripts.manipulability import (
    elbow_sweep,
    manipulability,
    nullspace_report,
    singularity_scan,
    task_jacobian,
    velocity_ellipsoid,
)

L1 = L2 = 0.5


@pytest.fixture(scope="module")
def arm2():
    return make_2r_arm(L1, L2)


@pytest.fixture(scope="module")
def arm7():
    return make_7dof_arm()


# ---------------------------------------------------------------------------
# 线性代数与旋转
# ---------------------------------------------------------------------------


def test_rotation_matrix_orthonormal():
    r = rotation_matrix(np.array([0.3, -0.5, 0.8]), 0.7)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0, abs=1e-12)


def test_rotation_axis_invariant():
    axis = np.array([1.0, 2.0, -0.5])
    axis = axis / np.linalg.norm(axis)
    r = rotation_matrix(axis, 1.1)
    assert np.allclose(r @ axis, axis, atol=1e-12)


def test_so3_log_roundtrip():
    for angle in (1e-9, 0.1, 1.0, np.pi - 1e-7):
        r = rotation_y(angle)
        v = so3_log(r)
        assert np.linalg.norm(v) == pytest.approx(angle, abs=1e-7)


def test_so3_log_zero():
    assert np.allclose(so3_log(np.eye(3)), 0.0)


def test_orientation_error_zero():
    r = rotation_z(0.4)
    assert np.allclose(orientation_error(r, r), 0.0, atol=1e-12)


def test_condition_number_singular():
    assert condition_number(np.zeros((2, 2))) == pytest.approx(float("inf"))


def test_joint_limit_cost_min_at_midpoint():
    lo, hi = np.array([0.0, 0.0]), np.array([1.0, 1.0])
    assert joint_limit_cost(np.array([0.5, 0.5]), lo, hi) < joint_limit_cost(
        np.array([0.5, 0.99]), lo, hi
    )


# ---------------------------------------------------------------------------
# 正运动学 / 雅可比
# ---------------------------------------------------------------------------


def test_joint_zero_axis_rejected():
    with pytest.raises(ValueError):
        Joint("bad", np.zeros(3), np.zeros(3))


def test_joint_limits_validation():
    with pytest.raises(ValueError):
        Joint("bad", np.zeros(3), np.array([0.0, 0.0, 1.0]), 1.0, 0.0)
    chain = make_2r_arm(0.5, 0.5)
    assert np.allclose(chain.q_mid, 0.0)


def test_chain_requires_joints():
    with pytest.raises(ValueError):
        SerialChain("empty", [], np.array([0.1, 0.0, 0.0]))


def test_fk_matches_analytic(arm2):
    for q0 in (-2.0, 0.0, 0.6):
        for q1 in (-1.0, 0.0, 1.4):
            p = arm2.forward(np.array([q0, q1])).position
            expect = [
                L1 * np.cos(q0) + L2 * np.cos(q0 + q1),
                L1 * np.sin(q0) + L2 * np.sin(q0 + q1),
                0.0,
            ]
            assert np.allclose(p, expect, atol=1e-12)


def test_jacobian_matches_numeric(arm2):
    q = np.array([0.4, 0.9])
    jac = arm2.jacobian(q)[:2]
    h = 1e-7
    numeric = np.zeros((2, 2))
    for i in range(2):
        plus, minus = q.copy(), q.copy()
        plus[i] += h
        minus[i] -= h
        numeric[:, i] = (
            arm2.forward(plus).position - arm2.forward(minus).position
        )[:2] / (2.0 * h)
    assert np.allclose(jac, numeric, atol=1e-6)


def test_dof_and_arm_span(arm2, arm7):
    assert arm2.dof == 2
    assert arm7.dof == 7
    assert arm2.q_mid.shape == (2,)
    assert arm7.q_mid.shape == (7,)
    # 2R 臂完全伸直时末端应落在 1.0 m 处。
    arm2_span = arm2.forward(np.array([0.0, 0.0])).position[0]
    assert arm2_span == pytest.approx(1.0, abs=1e-12)


def test_bad_q_dimension(arm2):
    with pytest.raises(ValueError):
        arm2.forward(np.zeros(3))


# ---------------------------------------------------------------------------
# 可操作性与奇异值
# ---------------------------------------------------------------------------


def test_manipulability_matches_closed_form(arm2):
    for elbow in (0.2, 0.7, 1.2, 2.0):
        q = np.array([0.3, elbow])
        value = manipulability(task_jacobian(arm2, q, rows=(0, 1))).value
        assert value == pytest.approx(abs(L1 * L2 * np.sin(elbow)), rel=1e-12)


def test_manipulability_max_at_right_angle(arm2):
    sweep = elbow_sweep(arm2, np.linspace(0.05, np.pi - 0.05, 101), L1, L2)
    idx = int(np.argmax(sweep["numeric"]))
    assert sweep["elbow"][idx] == pytest.approx(np.pi / 2.0, abs=0.05)


def test_manipulability_zero_when_straight(arm2):
    assert manipulability(task_jacobian(arm2, [0.0, 0.0], rows=(0, 1))).value == pytest.approx(0.0)


def test_3d_jacobian_is_always_singular(arm2):
    # 平面机构的 3 维位置雅可比 z 行恒为 0，所以所有位形都“奇异”
    # （σ_min 只有浮点舍入量级）。
    for elbow in (0.3, 1.0, 2.0):
        measure = manipulability(task_jacobian(arm2, [0.0, elbow], rows=(0, 1, 2)))
        assert measure.sigma_min < 1e-6


def test_velocity_ellipsoid_semiaxes(arm2):
    jac = task_jacobian(arm2, [0.0, 1.0], rows=(0, 1))
    ell = velocity_ellipsoid(jac)
    assert np.allclose(ell["semi_axes"], np.linalg.svd(jac, compute_uv=False))
    assert ell["points"].shape == (2, 121)


def test_pseudoinverse_step_grows_near_singularity(arm2):
    rows = singularity_scan(arm2, [0.2, 0.1, 0.05, 0.02, 0.01])
    steps = [row["step_pinv"] for row in rows]
    assert all(b > a for a, b in zip(steps, steps[1:]))
    # σ_min 减半，伪逆步长应基本翻倍。
    assert steps[-1] / steps[-2] == pytest.approx(2.0, rel=0.05)


def test_dls_step_bounded_near_singularity(arm2):
    rows = singularity_scan(arm2, [0.2, 0.05, 0.01, 0.005])
    steps = [row["step_dls"] for row in rows]
    assert steps[-1] < steps[0]
    assert all(b <= a for a, b in zip(steps, steps[1:]))


def test_nullspace_projection_does_not_move_ee(arm7):
    report = nullspace_report(arm7, arm7.q_mid)
    assert report["rank"] == 6
    assert report["nullity"] == 1
    assert report["end_effector_motion"] < 1e-9
    assert report["dq_norm"] > 1e-6


# ---------------------------------------------------------------------------
# 逆运动学
# ---------------------------------------------------------------------------


def test_position_ik_2r(arm2):
    target = np.array([0.4, 0.3, 0.0])
    result = solve_position_ik(arm2, target, rows=(0, 1))
    assert result.success
    assert result.error < 1e-6


def test_position_ik_unreachable(arm2):
    result = solve_position_ik(arm2, np.array([1.4, 0.0, 0.0]), rows=(0, 1))
    assert not result.success
    assert result.error > 0.1


def test_pose_ik_7dof_reachable(arm7):
    rng = np.random.default_rng(11)
    q_true = rng.uniform(
        arm7.lower + 0.25 * (arm7.upper - arm7.lower),
        arm7.upper - 0.25 * (arm7.upper - arm7.lower),
    )
    fk = arm7.forward(q_true)
    result = solve_pose_ik(arm7, fk.position, fk.rotation)
    assert result.success
    assert result.error < 1e-6


def test_pose_ik_unreachable(arm7):
    result = solve_pose_ik(arm7, np.array([2.0, 2.0, 2.0]), np.eye(3),
                           max_iter=60)
    assert not result.success
    assert result.error > 0.1


def test_pose_ik_2r_cannot_do_full_pose(arm2):
    result = solve_pose_ik(arm2, np.array([0.5, 0.5, 0.0]), rotation_z(0.3),
                           max_iter=100)
    assert not result.success


def test_pose_ik_respects_limits(arm7):
    rng = np.random.default_rng(2)
    q_true = rng.uniform(arm7.lower, arm7.upper)
    fk = arm7.forward(q_true)
    result = solve_pose_ik(arm7, fk.position, fk.rotation, q0=q_true + 0.2)
    assert arm7.lower[0] - 1e-9 <= result.q[0] <= arm7.upper[0] + 1e-9
    assert np.all(result.q >= arm7.lower - 1e-9)
    assert np.all(result.q <= arm7.upper + 1e-9)


def test_pose_ik_rejects_bad_method(arm2):
    with pytest.raises(ValueError):
        solve_pose_ik(arm2, np.zeros(3), np.eye(3), method="magic")


def test_pose_ik_wrong_q0_dimension(arm2):
    with pytest.raises(ValueError):
        solve_pose_ik(arm2, np.zeros(3), np.eye(3), q0=np.zeros(5))


def test_circle_tracking_converges(arm7):
    result = track_circle(arm7, (0.25, 0.0, 0.0), 0.12, n_points=12, plane="xz")
    assert result["success"].all()
    assert result["errors"].max() < 1e-6


def test_reference_circle_pose_on_circle(arm7):
    center = np.array([0.25, 0.0, 0.0])
    for angle in (0.0, 1.0, 3.0):
        p, r, q = reference_circle_pose(arm7, center, 0.12, angle, "xz")
        assert np.linalg.norm(p - center) == pytest.approx(0.12, abs=1e-12)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert q.shape == (7,)


def test_circle_pose_is_reachable(arm7):
    p, r, _ = reference_circle_pose(arm7, (0.25, 0, 0), 0.12, 1.2, "xz")
    result = solve_pose_ik(arm7, p, r)
    assert result.success


def test_joint_limit_gradient_points_inward():
    from scripts.linalg import joint_limit_gradient

    lo, hi = np.array([-1.0, -1.0]), np.array([1.0, 1.0])
    grad = joint_limit_gradient(np.array([0.95, 0.0]), lo, hi)
    assert grad[0] < 0.0  # 靠近上限 -> 往小推
    assert abs(grad[1]) < 1e-6
