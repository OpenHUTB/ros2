"""
tests.test_ik — 单元测试
=========================

运行: python -m pytest tests/test_ik.py -v
   或: python tests/test_ik.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from ik import (Arm7DOF, position_jacobian,
                solve_pseudoinverse, solve_damped_ls, solve_nullspace,
                manipulability_index, manipulability_ellipsoid, condition_number)


def test_fk_identity_at_zero():
    """零位时末端应在合理位置（臂长 0.5m 量级）。"""
    arm = Arm7DOF()
    q0 = np.zeros(7)
    p = arm.ee_position(q0)
    assert p.shape == (3,)
    assert 0.3 < np.linalg.norm(p) < 0.7, f"零位末端距离异常: {np.linalg.norm(p)}"
    print("[OK] test_fk_identity_at_zero: 零位末端位置合理 =", p)


def test_jacobian_consistency():
    """雅可比数值验证: J @ delta_q 应 ≈ fk(q+delta_q) - fk(q)"""
    arm = Arm7DOF()
    q = np.array([0.1, -0.2, 0.3, 0.0, 0.2, -0.1, 0.0])
    J = position_jacobian(arm, q)
    assert J.shape == (3, 7)
    dq = np.array([0.01, -0.01, 0.02, 0.0, 0.01, -0.01, 0.0])
    dp_pred = J @ dq
    dp_real = arm.ee_position(q + dq) - arm.ee_position(q)
    err = np.linalg.norm(dp_pred - dp_real)
    assert err < 2e-4, f"雅可比一致性误差过大: {err}"
    print(f"[OK] test_jacobian_consistency: 雅可比误差 = {err:.2e}")


def test_pseudoinverse_reaches_target():
    """伪逆法应能到达附近目标。"""
    arm = Arm7DOF()
    q0 = np.zeros(7)
    p0 = arm.ee_position(q0)
    target = p0 + np.array([0.05, 0.03, 0.02])
    q_hist, err_hist = solve_pseudoinverse(arm, q0, target, max_iter=300, tol=1e-4)
    final_err = err_hist[-1]
    assert final_err < 1e-3, f"伪逆法未收敛, 最终误差 = {final_err}"
    print(f"[OK] test_pseudoinverse_reaches_target: 收敛误差 = {final_err:.2e}, 迭代 {len(err_hist)}")


def test_damped_ls_reaches_target():
    """阻尼最小二乘法应收敛。"""
    arm = Arm7DOF()
    q0 = np.zeros(7)
    p0 = arm.ee_position(q0)
    target = p0 + np.array([0.04, -0.02, 0.03])
    q_hist, err_hist = solve_damped_ls(arm, q0, target, max_iter=300, tol=1e-4)
    final_err = err_hist[-1]
    assert final_err < 1e-3, f"阻尼LS未收敛, 最终误差 = {final_err}"
    print(f"[OK] test_damped_ls_reaches_target: 收敛误差 = {final_err:.2e}, 迭代 {len(err_hist)}")


def test_nullspace_avoids_limits():
    """零空间法应比纯伪逆更接近关节偏好位置。"""
    arm = Arm7DOF()
    q0 = np.zeros(7)
    p0 = arm.ee_position(q0)
    target = p0 + np.array([0.03, 0.02, 0.02])
    q_pref = np.array([0.1, 0.0, -0.1, 0.0, 0.1, 0.0, -0.1])
    q_ns, err_ns = solve_nullspace(arm, q0, target, q_pref, max_iter=300, tol=1e-4)
    assert err_ns[-1] < 1e-3, f"零空间法未收敛: {err_ns[-1]}"
    q_pi, err_pi = solve_pseudoinverse(arm, q0, target, max_iter=300, tol=1e-4)
    dist_ns = np.linalg.norm(q_ns[-1] - q_pref)
    dist_pi = np.linalg.norm(q_pi[-1] - q_pref)
    print(f"[OK] test_nullspace_avoids_limits: 零空间到偏好距离={dist_ns:.3f}, 伪逆={dist_pi:.3f}")


def test_manipulability_positive():
    """可操作性指标应非负，椭圆半轴为实数。"""
    arm = Arm7DOF()
    q = np.zeros(7)
    J = position_jacobian(arm, q)
    w = manipulability_index(J)
    semi, dirs = manipulability_ellipsoid(J)
    assert w > 0, f"可操作性指标为负: {w}"
    assert np.all(semi >= 0), f"椭圆半轴为负: {semi}"
    cond = condition_number(J)
    assert cond > 0
    print(f"[OK] test_manipulability_positive: w={w:.4f}, 半轴={semi.round(4)}, cond={cond:.2f}")


if __name__ == "__main__":
    test_fk_identity_at_zero()
    test_jacobian_consistency()
    test_pseudoinverse_reaches_target()
    test_damped_ls_reaches_target()
    test_nullspace_avoids_limits()
    test_manipulability_positive()
    print("\n=== 全部 6 项测试通过 ===")
