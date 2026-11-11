"""参考轨迹：正弦跟踪段与 20 deg -> 50 deg 阶跃段。"""

from __future__ import annotations

import math

import numpy as np

from .config import TrajectoryConfig


def make_time_vector(duration: float, dt: float) -> np.ndarray:
    """生成 0, dt, 2dt, ... 的时间向量，保证覆盖整个时长。"""
    if duration <= 0:
        raise ValueError("duration 必须 > 0")
    if dt <= 0:
        raise ValueError("dt 必须 > 0")
    n = int(round(duration / dt)) + 1
    return np.arange(n, dtype=float) * dt


def sine_reference(
    t: np.ndarray | float,
    bias_deg: float,
    amplitude_deg: float,
    frequency_hz: float,
) -> np.ndarray | float:
    """theta_des(t) = bias + amplitude * sin(2*pi*f*t)，单位 deg。"""
    if frequency_hz <= 0:
        raise ValueError("frequency_hz 必须 > 0")
    omega = 2.0 * math.pi * frequency_hz
    values = bias_deg + amplitude_deg * np.sin(omega * np.asarray(t, dtype=float))
    if np.isscalar(t):
        return float(values)
    return values


def step_reference(
    t: np.ndarray | float,
    start_deg: float,
    target_deg: float,
    step_time: float,
) -> np.ndarray | float:
    """step_time 之前保持 start，之后为目标 target 的阶跃。"""
    if step_time < 0:
        raise ValueError("step_time 必须 >= 0")
    time = np.asarray(t, dtype=float)
    values = np.where(time < step_time, start_deg, target_deg).astype(float)
    if np.isscalar(t):
        return float(values)
    return values


def sine_trajectory(t: np.ndarray, cfg: TrajectoryConfig) -> np.ndarray:
    """按配置生成正弦参考角。"""
    return np.asarray(
        sine_reference(
            t,
            cfg.sine_bias_deg,
            cfg.sine_amplitude_deg,
            cfg.sine_frequency_hz,
        ),
        dtype=float,
    )


def step_trajectory(t: np.ndarray, cfg: TrajectoryConfig) -> np.ndarray:
    """按配置生成阶跃参考角。"""
    return np.asarray(
        step_reference(t, cfg.step_start_deg, cfg.step_target_deg, cfg.step_time),
        dtype=float,
    )
