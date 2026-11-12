"""角度与坐标变换工具（无第三方依赖）。"""
from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


def wrap_angle(angle: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    return (angle + math.pi) % TWO_PI - math.pi


def body_to_world(angle: float, vx: float, vy: float) -> tuple[float, float]:
    """船体系向量转世界系。"""
    c, s = math.cos(angle), math.sin(angle)
    return c * vx - s * vy, s * vx + c * vy


def world_to_body(angle: float, vx: float, vy: float) -> tuple[float, float]:
    """世界系向量转船体系。"""
    c, s = math.cos(angle), math.sin(angle)
    return c * vx + s * vy, -s * vx + c * vy


def unit(angle: float) -> tuple[float, float]:
    return math.cos(angle), math.sin(angle)


def clamp(value: float, low: float, high: float) -> float:
    if high < low:
        raise ValueError("high 必须不小于 low")
    return low if value < low else high if value > high else value
