#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参考路径生成器：直线、圆弧、正弦曲线、双移线以及从文件读取航点。

所有生成器返回 `ReferencePath`，供控制器与仿真器使用。
"""

from __future__ import annotations

import math
import os
from typing import List, Tuple

import numpy as np

from .geometry import ReferencePath


def straight_line(length: float = 60.0, step: float = 0.5, heading: float = 0.0) -> ReferencePath:
    """沿给定航向的直线参考路径。"""
    s = np.arange(0.0, length + 1e-9, step)
    x = s * math.cos(heading)
    y = s * math.sin(heading)
    return ReferencePath(np.column_stack([x, y]))


def circular_arc(radius: float = 15.0, sweep_deg: float = 360.0, step_deg: float = 1.0) -> ReferencePath:
    """定半径圆弧。曲率恒为 1/radius，用来检验稳态横向偏差。"""
    if radius <= 0.0:
        raise ValueError("半径必须为正")
    sweep = math.radians(sweep_deg)
    # 弧长步长换算成角度步长，保证点间距接近 step（这里按 0.5 m 取默认）
    angles = np.arange(0.0, sweep + 1e-9, math.radians(step_deg))
    x = radius * np.sin(angles)
    y = radius * (1.0 - np.cos(angles))
    closed = abs(sweep_deg - 360.0) < 1e-6
    return ReferencePath(np.column_stack([x, y]), closed=closed)


def sine_wave(amplitude: float = 3.0, wavelength: float = 30.0,
              length: float = 90.0, step: float = 0.25) -> ReferencePath:
    """正弦曲线，用于检验连续变曲率下的跟踪能力。"""
    x = np.arange(0.0, length + 1e-9, step)
    y = amplitude * np.sin(2.0 * math.pi * x / wavelength)
    return ReferencePath(np.column_stack([x, y]))


def double_lane_change(length: float = 120.0, width: float = 3.5,
                       step: float = 0.25) -> ReferencePath:
    """双移线（ISO 3888-1 风格的简化版），横向加速度变化剧烈。"""
    x = np.arange(0.0, length + 1e-9, step)
    u = x / length
    y = width * 0.5 * (
        np.tanh(6.0 * (u - 0.2))
        - np.tanh(6.0 * (u - 0.4))
        - np.tanh(6.0 * (u - 0.6))
        + np.tanh(6.0 * (u - 0.8))
    )
    return ReferencePath(np.column_stack([x, y]))


def from_waypoints(points: List[Tuple[float, float]] | np.ndarray, closed: bool = False) -> ReferencePath:
    """从显式航点构造参考路径。"""
    return ReferencePath(np.asarray(points, dtype=float), closed=closed)


def from_csv(path: str, closed: bool = False) -> ReferencePath:
    """从 CSV 读取航点，支持有无表头，取前两列作为 x、y。"""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"航点文件不存在: {path}")
    data = np.genfromtxt(path, delimiter=",", comments="#")
    if data.ndim != 2 or data.shape[1] < 2:
        raise ValueError("CSV 至少需要两列 (x, y)")
    return ReferencePath(data[:, :2], closed=closed)


SCENARIOS = {
    "straight": lambda: straight_line(),
    "circle": lambda: circular_arc(),
    "sine": lambda: sine_wave(),
    "double_lane_change": lambda: double_lane_change(),
}


def build_scenario(name: str) -> ReferencePath:
    """按名称构造内置场景。"""
    if name not in SCENARIOS:
        raise KeyError(f"未知场景 '{name}'，可选: {', '.join(SCENARIOS)}")
    return SCENARIOS[name]()
