#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景定义：正弦中心线参考轨迹与位姿偏差计算。"""

from __future__ import annotations

import math

AMPLITUDE = 0.7
WAVE_K = 0.40
X_MIN, X_MAX = 0.0, 14.0


def centerline_y(x: float) -> float:
    return AMPLITUDE * math.sin(WAVE_K * x)


def centerline_slope(x: float) -> float:
    return AMPLITUDE * WAVE_K * math.cos(WAVE_K * x)


def lateral_offset(x: float, y: float) -> float:
    """机体相对中心线的横向偏移（右为正）。"""
    return y - centerline_y(x)


def heading_error(x: float, yaw: float) -> float:
    """机体航向相对中心线切线的偏航误差。"""
    desired = math.atan2(centerline_slope(x), 1.0)
    err = desired - yaw
    return math.atan2(math.sin(err), math.cos(err))
