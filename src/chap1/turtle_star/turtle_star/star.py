#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五角星几何与绘制顺序（纯函数，便于单元测试）。

对应文档 `docs/chapter/chap1/turtle_star.md`：

* 五角星外角 144°，循环 5 次「直行 L → 原地左转 144°」即可闭合；
* 几何上等价于在正五边形顶点间跳跃，总转角 = 5 × 144° = 720° = 4π。
"""

from __future__ import annotations

import math

TURN_ANGLE = math.radians(144.0)
EDGES = 5
TOTAL_TURN = TURN_ANGLE * EDGES


def vertices(side_length: float) -> list[tuple[float, float]]:
    """从原点朝 +x 出发，返回五角星的 5 个顶点（含回到起点前的最后一个点）。"""
    if side_length <= 0.0:
        raise ValueError("边长必须 > 0")
    x = y = 0.0
    heading = 0.0
    points = [(x, y)]
    for _ in range(EDGES):
        x += side_length * math.cos(heading)
        y += side_length * math.sin(heading)
        points.append((x, y))
        heading += TURN_ANGLE
    return points


def closure_error(side_length: float) -> float:
    """走完 5 条边后回到起点的距离误差（应为 0）。"""
    points = vertices(side_length)
    return math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1])


def edge_duration(side_length: float, linear_speed: float) -> float:
    """直行一条边所需时间 [s]。"""
    if linear_speed <= 0.0:
        raise ValueError("线速度必须 > 0")
    return side_length / linear_speed


def turn_duration(angular_speed: float) -> float:
    """原地转 144° 所需时间 [s]。"""
    if angular_speed <= 0.0:
        raise ValueError("角速度必须 > 0")
    return TURN_ANGLE / angular_speed


def palette(count: int = EDGES) -> list[tuple[int, int, int]]:
    """五条边使用的 RGB 颜色（0~255）。"""
    colors = [
        (255, 0, 0),
        (255, 165, 0),
        (255, 255, 0),
        (0, 200, 0),
        (0, 128, 255),
    ]
    return colors[:count]

def wrap_to_pi(angle):
    """把角度归一化到 (-pi, pi]。"""
    value = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if value <= 0.0:
        value += 2.0 * math.pi
    return value - math.pi


class TurnAccumulator:
    """累计转过的角度（有符号 + 死区，处理 +/-pi 跳变与静止抖动）。"""

    def __init__(self, target=TURN_ANGLE, tolerance=1e-3, deadband=1e-3):
        if target <= 0.0:
            raise ValueError("目标角度必须 > 0")
        if deadband < 0.0:
            raise ValueError("死区不能为负")
        self.target = float(target)
        self.tolerance = float(tolerance)
        self.deadband = float(deadband)
        self.total = 0.0
        self._last = None

    def update(self, theta):
        theta = float(theta)
        if self._last is None:
            self._last = theta
            return self.total
        delta = wrap_to_pi(theta - self._last)
        self._last = theta
        if abs(delta) >= self.deadband:
            self.total += delta
        return self.total

    @property
    def done(self):
        return abs(self.total) >= self.target - self.tolerance

    def reset(self):
        self.total = 0.0
        self._last = None
