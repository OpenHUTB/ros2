"""
ik.manipulability — Yoshikawa 可操作性椭圆
=============================================

w = sqrt(det(J J^T))  （Yoshikawa 指标）
椭圆半轴 = svd(J J^T) 的特征值平方根
"""
import numpy as np
from .robot import Arm7DOF
from .jacobian import position_jacobian


def manipulability_index(J):
    """Yoshikawa 可操作性指标 w = sqrt(det(J J^T))。"""
    JJt = J @ J.T
    return float(np.sqrt(max(np.linalg.det(JJt), 0.0)))


def manipulability_ellipsoid(J):
    """返回可操作性椭圆的半轴长和方向。

    返回 (semi_axes (3,), directions (3x3))
    """
    JJt = J @ J.T
    eigvals, eigvecs = np.linalg.eigh(JJt)
    eigvals = np.clip(eigvals, 0, None)
    semi = np.sqrt(eigvals)
    return semi, eigvecs


def condition_number(J):
    """雅可比条件数（最大奇异值/最小奇异值）。"""
    s = np.linalg.svd(J, compute_uv=False)
    s = s[s > 1e-10]
    if len(s) == 0:
        return float('inf')
    return float(s.max() / s.min())
