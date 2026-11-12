#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""小海龟画圆的运动学与转角累计（纯函数，便于单元测试）。

对应文档 `docs/chapter/chap1/turtle_circle.md`：

* 圆周运动：``omega = v / r``，半径由线速度与角速度之比决定；
* 用位姿反馈累计海龟转过的角度，累计到 2π 就停，保证画满一圈不多不少。

转角用**有符号**累加（取最短角差），这样传感器抖动会相互抵消；
再加一个死区，静止时的浮点噪声不会被当成转动。
"""

from __future__ import annotations

import math

FULL_TURN = 2.0 * math.pi
DEFAULT_DEADBAND = 1e-3


def command_for_circle(radius: float, linear_speed: float) -> tuple[float, float]:
    """给定半径 ``r`` 与线速度 ``v``，返回 ``(v, omega)``。"""
    if radius <= 0.0:
        raise ValueError("半径必须 > 0")
    if linear_speed <= 0.0:
        raise ValueError("线速度必须 > 0")
    return float(linear_speed), float(linear_speed) / float(radius)


def wrap_to_pi(angle: float) -> float:
    """把角度归一化到 (-π, π]。"""
    value = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if value <= 0.0:
        value += 2.0 * math.pi
    return value - math.pi


class TurnAccumulator:
    """累计海龟转过的角度（有符号 + 死区，处理 ±π 跳变与静止抖动）。"""

    def __init__(self, target: float = FULL_TURN,
                 tolerance: float = 1e-3,
                 deadband: float = DEFAULT_DEADBAND):
        if target <= 0.0:
            raise ValueError("目标角度必须 > 0")
        if deadband < 0.0:
            raise ValueError("死区不能为负")
        self.target = float(target)
        self.tolerance = float(tolerance)
        self.deadband = float(deadband)
        self.total = 0.0
        self._last = None

    def update(self, theta: float) -> float:
        """喂入当前朝向角，返回累计转角（弧度，带符号）。"""
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
    def done(self) -> bool:
        return abs(self.total) >= self.target - self.tolerance

    @property
    def progress(self) -> float:
        return min(1.0, abs(self.total) / self.target)

    def reset(self) -> None:
        self.total = 0.0
        self._last = None
