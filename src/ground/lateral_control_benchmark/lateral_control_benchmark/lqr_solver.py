#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""连续代数 Riccati 方程（CARE）求解器。

本模块自带一个纯 NumPy 实现，使整个功能包只依赖 ``numpy``（Ubuntu 20.04 +
ROS Noetic 的 Python 3.8 自带 numpy），避免额外安装 ``scipy``——这是本模块
相对其他实现的一个刻意取舍：少一个依赖，就少一个"装不上"的失败点。

求解方法：CARE
    A^T P + P A - P B R^-1 B^T P + Q = 0
等价于求 Hamiltonian 矩阵
    H = [[A, -B R^-1 B^T], [-Q, -A^T]]
的稳定不变子空间。取 H 的特征向量中实部为负的那些列 [U; V]，
则 P = V U^-1 即稳定解。
"""

from __future__ import annotations

import numpy as np


def solve_care(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """求解连续代数 Riccati 方程，返回半正定解 P。

    参数
    ----
    a : (n, n) 系统矩阵
    b : (n, m) 输入矩阵
    q : (n, n) 状态权重（对称半正定）
    r : (m, m) 控制权重（对称正定）

    返回
    ----
    p : (n, n) 对称半正定解

    异常
    ----
    ValueError : 当 Hamiltonian 矩阵不存在稳定不变子空间时抛出
    """
    a = np.atleast_2d(np.asarray(a, dtype=float))
    b = np.atleast_2d(np.asarray(b, dtype=float))
    q = np.atleast_2d(np.asarray(q, dtype=float))
    r = np.atleast_2d(np.asarray(r, dtype=float))
    n = a.shape[0]
    if b.shape[0] != n or q.shape != (n, n) or r.shape != (b.shape[1], b.shape[1]):
        raise ValueError("CARE 输入维度不匹配")

    r_inv = np.linalg.inv(r)
    h = np.block([[a, -b @ r_inv @ b.T],
                  [-q, -a.T]])

    eigvals, eigvecs = np.linalg.eig(h)
    stable = np.real(eigvals) < 0.0
    if int(np.sum(stable)) != n:
        raise ValueError("Hamiltonian 矩阵没有足够的稳定特征值，无法求 CAREd 解")

    # 数值上更稳的做法：按实部从小到大排序后取前 n 个
    order = np.argsort(np.real(eigvals))
    chosen = order[:n]
    u = eigvecs[:n, chosen]
    v = eigvecs[n:, chosen]
    p = np.real(v @ np.linalg.inv(u))
    return 0.5 * (p + p.T)          # 强制对称，消除数值误差
