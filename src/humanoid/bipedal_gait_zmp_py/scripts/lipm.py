"""线性倒立摆（LIPM）基础动力学。

约定 x 为前进方向，p 为 ZMP 位置。所有长度单位为 m，时间单位为 s。
"""

from __future__ import annotations

import math

import numpy as np

G = 9.81
Z_C = 0.8
OMEGA = math.sqrt(G / Z_C)


def lipm_acceleration(x: float, p: float, omega: float = OMEGA) -> float:
    """由 LIPM 方程 x_ddot = omega^2 (x - p) 计算质心加速度。"""
    return omega * omega * (x - p)


def zmp_from_acceleration(x: float, x_ddot: float, omega: float = OMEGA) -> float:
    """ZMP 反算：p = x - x_ddot / omega^2。"""
    return x - x_ddot / (omega * omega)


def lipm_step(x: float, v: float, p: float, dt: float, omega: float = OMEGA) -> tuple[float, float]:
    """恒定 ZMP 下 LIPM 的精确一步积分（零阶保持 p）。"""
    c = math.cosh(omega * dt)
    s = math.sinh(omega * dt)
    x_next = (x - p) * c + (v / omega) * s + p
    v_next = (x - p) * omega * s + v * c
    return x_next, v_next


def analytic_constant_zmp(
    x0: float,
    v0: float,
    p: float,
    t: np.ndarray | float,
    omega: float = OMEGA,
) -> tuple[np.ndarray, np.ndarray]:
    """恒定 ZMP 的解析解，返回 (x(t), v(t))。"""
    tt = np.asarray(t, dtype=float)
    wt = omega * tt
    dx = x0 - p
    x = dx * np.cosh(wt) + (v0 / omega) * np.sinh(wt) + p
    v = dx * omega * np.sinh(wt) + v0 * np.cosh(wt)
    return x, v


def integrate_constant_zmp(
    x0: float,
    v0: float,
    p: float,
    duration: float,
    dt: float = 1e-3,
    omega: float = OMEGA,
) -> tuple[float, float]:
    """数值积分恒定 ZMP 的 LIPM，主要用于测试解析解。"""
    n = int(round(duration / dt))
    x, v = float(x0), float(v0)
    for _ in range(n):
        x, v = lipm_step(x, v, p, dt, omega=omega)
    return x, v


def second_derivative_error(
    x0: float,
    v0: float,
    p: float,
    duration: float,
    dt: float = 2.5e-4,
    omega: float = OMEGA,
) -> float:
    """用解析轨迹的中心二阶差分核对 LIPM 方程，返回最大绝对残差。"""
    times = np.arange(0.0, duration + 0.5 * dt, dt)
    x, _ = analytic_constant_zmp(x0, v0, p, times, omega=omega)
    x_ddot = np.gradient(np.gradient(x, dt), dt)
    residual = x_ddot - omega * omega * (x - p)
    # 两端点的差分模板误差较大，取内部点更合理。
    return float(np.max(np.abs(residual[2:-2])))
