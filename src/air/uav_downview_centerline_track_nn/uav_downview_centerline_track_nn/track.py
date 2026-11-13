#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面参考标线（中心曲线）定义。

标线为 x 方向前进的一条平滑 S 形曲线，下视相机与循迹控制器都以
``reference_y`` 与 ``reference_tangent`` 作为期望轨迹基准。
"""

from __future__ import annotations

import math

AMPLITUDE = 1.1      # 横向幅值 m
WAVE_K = 0.55        # 空间角频率 rad/m
X_MIN = 0.0
X_MAX = 14.0


def reference_y(x: float) -> float:
    """标线在给定 x 处的横向坐标 y。"""
    return AMPLITUDE * math.sin(WAVE_K * x)


def reference_slope(x: float) -> float:
    """标线在给定 x 处的斜率 dy/dx。"""
    return AMPLITUDE * WAVE_K * math.cos(WAVE_K * x)


def reference_tangent(x: float) -> float:
    """标线切线相对 x 轴的航向角 rad（世界系 ENU）。"""
    return math.atan2(reference_slope(x), 1.0)


def lateral_error(x: float, y: float) -> float:
    """机体相对标线的横向误差：正值表示机体在标线左侧。"""
    return y - reference_y(x)
