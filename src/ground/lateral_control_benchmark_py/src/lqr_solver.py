"""纯 numpy 的连续代数 Riccati 方程（CARE）求解器。

利用 Hamiltonian 矩阵的稳定不变子空间：
    H = [[A, -B R^-1 B^T],
         [-Q, -A^T]]
若 V = [V1; V2] 张成 H 的 n 维稳定不变子空间，则 P = V2 V1^-1 为 CARE 的解。

不依赖 scipy。对于小维度（5 状态）特征分解足够快、精度足够。
"""

from __future__ import annotations

import numpy as np


def _symmetrize(matrix):
    return 0.5 * (matrix + matrix.T)


def solve_care_hamiltonian(A, B, Q, R):
    """求解连续 Riccati 方程，返回对称半正定矩阵 P。

    参数
    ----
    A : (n, n)
    B : (n, m)
    Q : (n, n)，对称半正定
    R : (m, m)，对称正定
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)

    n = A.shape[0]
    if A.shape != (n, n):
        raise ValueError("A must be square")
    if B.ndim != 2 or B.shape[0] != n:
        raise ValueError("B has incompatible shape")
    if Q.shape != (n, n) or R.shape[1] != B.shape[1] or R.shape[0] != B.shape[1]:
        raise ValueError("Q/R have incompatible shapes")
    if not (np.all(np.isfinite(A)) and np.all(np.isfinite(B))
            and np.all(np.isfinite(Q)) and np.all(np.isfinite(R))):
        raise ValueError("CARE inputs contain non-finite values")

    Q = _symmetrize(Q)
    R = _symmetrize(R)
    R_inv = np.linalg.inv(R)
    G = B @ R_inv @ B.T
    H = np.block([[A, -G], [-Q, -A.T]])

    eigenvalues, eigenvectors = np.linalg.eig(H)
    # Hamiltonian 的特征值关于虚轴对称；取实部最小的 n 个作为稳定子空间。
    order = np.argsort(eigenvalues.real)
    stable = order[:n]
    V = eigenvectors[:, stable]
    V1 = V[:n, :]
    V2 = V[n:, :]

    # 用最小二乘代替显式逆，提升对接近虚轴特征值的数值鲁棒性。
    P = V2 @ np.linalg.pinv(V1)
    if np.iscomplexobj(P):
        if np.max(np.abs(P.imag)) > 1e-7:
            # 稳定子空间应当给出实矩阵；若虚部较大，用实部继续并做对称化。
            P = P.real
        else:
            P = P.real
    P = _symmetrize(np.asarray(P, dtype=float))
    K = R_inv @ B.T @ P

    residual = A.T @ P + P @ A - P @ B @ R_inv @ B.T @ P + Q
    residual_norm = float(np.linalg.norm(residual, ord="fro"))
    closed_loop = A - B @ K
    eigs = np.linalg.eigvals(closed_loop)
    return {
        "P": P,
        "K": K,
        "residual": residual,
        "residual_norm": residual_norm,
        "closed_loop_eigs": eigs,
        "hamiltonian_eigs": eigenvalues,
    }


def solve_care(A, B, Q, R):
    """兼容简写，只返回增益 K。"""
    return solve_care_hamiltonian(A, B, Q, R)["K"]


def is_stable(A_cl, tolerance: float = 1e-8) -> bool:
    eigs = np.linalg.eigvals(np.asarray(A_cl, dtype=float))
    return bool(np.all(eigs.real < -tolerance))
