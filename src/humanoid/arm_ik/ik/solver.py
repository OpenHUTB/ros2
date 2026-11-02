"""
ik.solver — 三种逆运动学求解器
================================

1. 伪逆法 (pseudoinverse):      q += lambda * J^+ * e
2. 阻尼最小二乘 (damped LS):    q += J^T (J J^T + lambda^2 I)^-1 e
3. 零空间投影 (nullspace):      q += J^+ e + (I - J^+ J) * q_pref
"""
import numpy as np
from .robot import Arm7DOF
from .jacobian import position_jacobian


def solve_pseudoinverse(arm: Arm7DOF, q0, target_pos,
                        max_iter=200, tol=1e-4, lr=0.5):
    """伪逆法逆解。返回 (q_history, err_history)。"""
    q = q0.copy()
    q_hist, err_hist = [q.copy()], []
    for _ in range(max_iter):
        p = arm.ee_position(q)
        e = target_pos - p
        err = np.linalg.norm(e)
        err_hist.append(err)
        if err < tol:
            break
        J = position_jacobian(arm, q)
        J_pinv = np.linalg.pinv(J)
        q = q + lr * J_pinv @ e
        q_hist.append(q.copy())
    return np.array(q_hist), np.array(err_hist)


def solve_damped_ls(arm: Arm7DOF, q0, target_pos,
                    max_iter=200, tol=1e-4, lr=0.5, lam=0.1):
    """阻尼最小二乘法（Levenberg-Marquardt 风格）。奇异点附近更稳定。"""
    q = q0.copy()
    q_hist, err_hist = [q.copy()], []
    for _ in range(max_iter):
        p = arm.ee_position(q)
        e = target_pos - p
        err = np.linalg.norm(e)
        err_hist.append(err)
        if err < tol:
            break
        J = position_jacobian(arm, q)
        A = J @ J.T + lam**2 * np.eye(3)
        delta_q = J.T @ np.linalg.solve(A, e)
        q = q + lr * delta_q
        q_hist.append(q.copy())
    return np.array(q_hist), np.array(err_hist)


def solve_nullspace(arm: Arm7DOF, q0, target_pos, q_pref,
                    max_iter=200, tol=1e-4, lr=0.5, w_ns=0.3):
    """零空间投影：主任务追末端位置，零空间里优化关节偏好（避开限位）。"""
    q = q0.copy()
    q_hist, err_hist = [q.copy()], []
    for _ in range(max_iter):
        p = arm.ee_position(q)
        e = target_pos - p
        err = np.linalg.norm(e)
        err_hist.append(err)
        if err < tol:
            break
        J = position_jacobian(arm, q)
        J_pinv = np.linalg.pinv(J)
        # 主任务
        dq_primary = J_pinv @ e
        # 零空间投影：把偏好关节方向投影到 J 的零空间
        N = np.eye(7) - J_pinv @ J
        dq_pref = q_pref - q
        q = q + lr * dq_primary + w_ns * (N @ dq_pref)
        q_hist.append(q.copy())
    return np.array(q_hist), np.array(err_hist)
