"""线性代数与数值工具。

本模块只依赖 numpy，提供旋转、SO(3) 对数映射、伪逆、阻尼最小二乘、
零空间投影、自适应阻尼和关节限位代价函数等基础工具。
"""

from __future__ import annotations

import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    """返回三维向量对应的反对称矩阵。"""
    v = np.asarray(v, dtype=float).reshape(3)
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=float,
    )


def rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues 公式：绕给定轴旋转 angle 弧度。"""
    axis = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError("旋转轴不能是零向量")
    axis = axis / norm
    K = skew(axis)
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def rotation_x(angle: float) -> np.ndarray:
    return rotation_matrix(np.array([1.0, 0.0, 0.0]), angle)


def rotation_y(angle: float) -> np.ndarray:
    return rotation_matrix(np.array([0.0, 1.0, 0.0]), angle)


def rotation_z(angle: float) -> np.ndarray:
    return rotation_matrix(np.array([0.0, 0.0, 1.0]), angle)


def homogeneous(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """由旋转和平移构造 4x4 齐次变换。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(p, dtype=float).reshape(3)
    return T


def so3_log(R: np.ndarray) -> np.ndarray:
    """SO(3) 对数映射，返回旋转向量（轴 * 角）。

    对接近 pi 的情形采用对称矩阵选取最大列，避免 sin(theta) 除零。
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    # 数值上把矩阵投影回 SO(3)，避免累计误差造成 trace 越界。
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt

    cos_theta = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    vee = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=float,
    )

    if theta < 1e-10:
        return 0.5 * vee

    if np.pi - theta < 1e-6:
        # 接近 pi 时 1/sin(theta) 不稳定，改用 (R + I)/2 = a a^T 提取轴。
        A = 0.5 * (R + np.eye(3))
        diag = np.maximum(np.diag(A), 0.0)
        k = int(np.argmax(diag))
        if np.linalg.norm(A[:, k]) < 1e-12:
            return np.zeros(3)
        axis = A[:, k] / np.linalg.norm(A[:, k])
        # 用非对角元素确定整体符号。
        if np.dot(axis, vee) < 0.0:
            axis = -axis
        return theta * axis

    return (theta / (2.0 * np.sin(theta))) * vee


def orientation_error(R_target: np.ndarray, R_current: np.ndarray) -> np.ndarray:
    """目标姿态相对当前姿态的旋转向量误差。"""
    return so3_log(np.asarray(R_target) @ np.asarray(R_current).T)


def pseudoinverse(J: np.ndarray, rcond: float = 1e-10) -> np.ndarray:
    """带奇异值截断的 Moore-Penrose 伪逆。"""
    J = np.asarray(J, dtype=float)
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    if s.size == 0 or s[0] <= 0.0:
        return np.zeros((J.shape[1], J.shape[0]), dtype=float)
    cutoff = rcond * s[0]
    s_inv = np.where(s > cutoff, 1.0 / s, 0.0)
    return (Vt.T * s_inv) @ U.T


def pinv_step(J: np.ndarray, error: np.ndarray, rcond: float = 1e-10) -> np.ndarray:
    """伪逆逆解步：dq = J^+ e。"""
    return pseudoinverse(J, rcond=rcond) @ np.asarray(error, dtype=float)


def dls_step(
    J: np.ndarray,
    error: np.ndarray,
    damping: float,
    rcond: float = 1e-10,
) -> np.ndarray:
    """阻尼最小二乘步：dq = J^T (J J^T + lambda^2 I)^-1 e。"""
    J = np.asarray(J, dtype=float)
    e = np.asarray(error, dtype=float)
    if damping <= 1e-12:
        return pinv_step(J, e, rcond=rcond)
    A = J @ J.T + (damping * damping) * np.eye(J.shape[0], dtype=float)
    try:
        y = np.linalg.solve(A, e)
    except np.linalg.LinAlgError:
        y = np.linalg.lstsq(A, e, rcond=None)[0]
    return J.T @ y


def nullspace_projector(J: np.ndarray, rcond: float = 1e-10) -> np.ndarray:
    """零空间投影矩阵 P = I - J^+ J。"""
    J = np.asarray(J, dtype=float)
    return np.eye(J.shape[1], dtype=float) - pseudoinverse(J, rcond=rcond) @ J


def nullspace_step(
    J: np.ndarray,
    error: np.ndarray,
    bias: np.ndarray,
    rcond: float = 1e-10,
) -> np.ndarray:
    """零空间投影步：dq = J^+ e + (I - J^+ J) z。"""
    P = nullspace_projector(J, rcond=rcond)
    return pinv_step(J, error, rcond=rcond) + P @ np.asarray(bias, dtype=float)


def adaptive_damping(
    sigma_min: float,
    threshold: float = 0.05,
    lambda_min: float = 1e-4,
    lambda_max: float = 0.2,
) -> float:
    """根据最小奇异值线性调节阻尼。

    sigma_min >= threshold 时使用 lambda_min（近似无阻尼）；
    sigma_min 从 threshold 降到 0 时，lambda 从 lambda_min 线性升到 lambda_max。
    """
    sigma = float(max(0.0, sigma_min))
    threshold = float(threshold)
    if threshold <= 0.0:
        return float(lambda_min)
    if sigma >= threshold:
        return float(lambda_min)
    ratio = 1.0 - sigma / threshold
    return float(lambda_min + ratio * (lambda_max - lambda_min))


def condition_number(J: np.ndarray, rcond: float = 1e-12) -> float:
    """矩阵条件数 sigma_max / sigma_min（奇异时返回 inf）。

    注意：numpy 的 svd 只返回 min(m, n) 个奇异值；当 m > n 时剩下的
    m - n 个奇异值恒为 0，必须在这里补上，否则会漏判奇异。
    """
    J = np.asarray(J, dtype=float)
    s = np.linalg.svd(J, compute_uv=False)
    if J.shape[0] > s.size:
        s = np.concatenate([s, np.zeros(J.shape[0] - s.size)])
    if s.size == 0 or s[0] <= 0.0:
        return float("inf")
    smin = float(s[-1])
    if smin <= rcond * float(s[0]):
        return float("inf")
    return float(s[0] / smin)


def singular_values(J: np.ndarray) -> np.ndarray:
    """按降序返回奇异值。"""
    return np.linalg.svd(np.asarray(J, dtype=float), compute_uv=False)


def ellipsoid_semiaxes(J: np.ndarray) -> np.ndarray:
    """速度椭球半轴长度（雅可比奇异值）。"""
    return singular_values(J)


def joint_limit_cost(
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    eps: float = 1e-9,
) -> float:
    """双曲型关节限位代价。

    归一化到 [0, 1] 后使用 1 / (x (1-x))，中点取最小值 4，越靠近限位越大。
    """
    q = np.asarray(q, dtype=float)
    lo = np.asarray(lower, dtype=float)
    hi = np.asarray(upper, dtype=float)
    span = np.maximum(hi - lo, eps)
    x = np.clip((q - lo) / span, eps, 1.0 - eps)
    return float(np.sum(1.0 / (x * (1.0 - x))))


def joint_limit_gradient(
    q: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    eps: float = 1e-9,
) -> np.ndarray:
    """双曲限位代价的解析梯度。

    对 c(x) = 1 / (x (1-x))，dc/dx = (2x-1) / (x² (1-x)²)；
    换回 q 后还要除以跨度 span。梯度**朝区间内部**：q 靠近上限时 x→1，
    (2x-1) > 0，因此取负号后 q 被往小推。
    """
    q = np.asarray(q, dtype=float)
    lo = np.asarray(lower, dtype=float)
    hi = np.asarray(upper, dtype=float)
    span = np.maximum(hi - lo, eps)
    x = np.clip((q - lo) / span, eps, 1.0 - eps)
    # -d/dq [1/(x(1-x))] = -(2x-1) / (x^2 (1-x)^2 span)
    return -(2.0 * x - 1.0) / (x * x * (1.0 - x) * (1.0 - x) * span)


def is_finite(*arrays: np.ndarray) -> bool:
    """检查一组数组是否全部有限。"""
    return all(np.all(np.isfinite(np.asarray(a, dtype=float))) for a in arrays)
