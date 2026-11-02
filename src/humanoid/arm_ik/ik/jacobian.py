"""
ik.jacobian — 几何雅可比（位置雅可比 3x7）
============================================

用数值差分计算位置雅可比：J = ∂p/∂q
"""
import numpy as np
from .robot import Arm7DOF


def position_jacobian(arm: Arm7DOF, q, eps=1e-6):
    """3x7 位置雅可比。J[:,i] = (p(q+e_i*eps) - p(q)) / eps"""
    p0 = arm.ee_position(q)
    J = np.zeros((3, 7))
    for i in range(7):
        qp = q.copy()
        qp[i] += eps
        J[:, i] = (arm.ee_position(qp) - p0) / eps
    return J


def full_jacobian(arm: Arm7DOF, q, eps=1e-6):
    """6x7 全雅可比（位置3 + 姿态3），姿态用数值差分绕轴旋转近似。"""
    p0, R0 = arm.ee_pose(q)
    J = np.zeros((6, 7))
    for i in range(7):
        qp = q.copy()
        qp[i] += eps
        pp, Rp = arm.ee_pose(qp)
        J[:3, i] = (pp - p0) / eps
        # 姿态差分: dR/dq ≈ [w]x R，w 用反对称矩阵提取
        dR = (Rp - R0) / eps
        # w_x = (dR @ R0.T) 的反对称部分
        W = dR @ R0.T
        w = np.array([W[2,1]-W[1,2], W[0,2]-W[2,0], W[1,0]-W[0,1]]) / 2.0
        J[3:, i] = w
    return J
