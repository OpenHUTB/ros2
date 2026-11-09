#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三次 Hermite 样条工具。

本项目的“三次样条”指**分段三次 Hermite 插值**：给定两端点的值与一阶导数，
可在过渡区间内构造 C1 连续的三次多项式，并给出解析导数供速度前馈使用。
注意这与自然三次样条（natural cubic spline，C2 连续、端点二阶导为零）不同，
主流程中没有使用自然三次样条。
"""

from typing import Sequence

import numpy as np


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


def hermite_scalar_derivative(t: float, t0: float, q0: float, dq0: float,
                              t1: float, q1: float, dq1: float) -> float:
    """单个通道三次 Hermite 插值对时间的解析导数。"""
    if t1 <= t0:
        return 0.0
    u = (t - t0) / (t1 - t0)
    u = max(0.0, min(1.0, u))
    dh00 = 6 * u ** 2 - 6 * u
    dh10 = 3 * u ** 2 - 4 * u + 1
    dh01 = -6 * u ** 2 + 6 * u
    dh11 = 3 * u ** 2 - 2 * u
    T = t1 - t0
    return (dh00 * q0 + dh10 * T * dq0 + dh01 * q1 + dh11 * T * dq1) / T


def hermite_vector(t: float, t0: float, q0: np.ndarray, dq0: np.ndarray,
                   t1: float, q1: np.ndarray, dq1: np.ndarray) -> np.ndarray:
    """多通道三次 Hermite 插值。"""
    return np.array([
        hermite_scalar(t, t0, q0[i], dq0[i], t1, q1[i], dq1[i])
        for i in range(len(q0))
    ])


def hermite_vector_derivative(t: float, t0: float, q0: np.ndarray,
                              dq0: np.ndarray, t1: float, q1: np.ndarray,
                              dq1: np.ndarray) -> np.ndarray:
    """多通道三次 Hermite 插值的一阶导数。"""
    return np.array([
        hermite_scalar_derivative(t, t0, q0[i], dq0[i], t1, q1[i], dq1[i])
        for i in range(len(q0))
    ])


def sample_spline(fn, t0: float, t1: float, samples: int = 100) -> tuple:
    """在 [t0, t1] 上等间隔采样。"""
    ts = np.linspace(t0, t1, samples)
    return ts, np.array([fn(t) for t in ts])
