#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三次样条插值工具。

包含两类：
1. 自然三次样条（natural cubic spline）：给定节点 (x_i, y_i)，构造 C2 连续曲线；
2. 三次 Hermite 样条段：给定两端点值/速度，用于步态切换时的 C1 平滑过渡。
"""

from typing import Callable, List, Sequence

import numpy as np


def natural_cubic_spline(x: Sequence[float], y: Sequence[float]) -> Callable[[float], float]:
    """返回自然三次样条求值函数（端点二阶导数为 0）。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if n < 2:
        raise ValueError("至少需要 2 个节点")
    if n == 2:
        slope = (y[1] - y[0]) / (x[1] - x[0])
        return lambda t: y[0] + slope * (t - x[0])

    h = np.diff(x)
    # 三对角方程组求二阶导数 M
    A = np.zeros((n, n))
    b = np.zeros(n)
    A[0, 0] = 1.0
    A[-1, -1] = 1.0
    for i in range(1, n - 1):
        A[i, i - 1] = h[i - 1]
        A[i, i] = 2.0 * (h[i - 1] + h[i])
        A[i, i + 1] = h[i]
        b[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])

    m = np.linalg.solve(A, b)

    def evaluate(t: float) -> float:
        t = float(t)
        if t <= x[0]:
            return float(y[0])
        if t >= x[-1]:
            return float(y[-1])
        idx = int(np.searchsorted(x, t) - 1)
        idx = max(0, min(idx, n - 2))
        hi = h[idx]
        a = (x[idx + 1] - t) / hi
        bb = (t - x[idx]) / hi
        return float(
            a * y[idx] + bb * y[idx + 1]
            + ((a ** 3 - a) * m[idx] + (bb ** 3 - bb) * m[idx + 1]) * (hi ** 2) / 6.0
        )

    return evaluate


def hermite_scalar(t: float, t0: float, q0: float, dq0: float,
                   t1: float, q1: float, dq1: float) -> float:
    """单个通道的三次 Hermite 插值。"""
    if t1 <= t0:
        return q1
    u = (t - t0) / (t1 - t0)
    u = max(0.0, min(1.0, u))
    h00 = 2 * u ** 3 - 3 * u ** 2 + 1
    h10 = u ** 3 - 2 * u ** 2 + u
    h01 = -2 * u ** 3 + 3 * u ** 2
    h11 = u ** 3 - u ** 2
    T = t1 - t0
    return h00 * q0 + h10 * T * dq0 + h01 * q1 + h11 * T * dq1


def hermite_vector(t: float, t0: float, q0: np.ndarray, dq0: np.ndarray,
                   t1: float, q1: np.ndarray, dq1: np.ndarray) -> np.ndarray:
    """多通道三次 Hermite 插值。"""
    return np.array([
        hermite_scalar(t, t0, q0[i], dq0[i], t1, q1[i], dq1[i])
        for i in range(len(q0))
    ])


def sample_spline(fn: Callable[[float], float], t0: float, t1: float,
                  samples: int = 100) -> tuple:
    ts = np.linspace(t0, t1, samples)
    return ts, np.array([fn(t) for t in ts])
