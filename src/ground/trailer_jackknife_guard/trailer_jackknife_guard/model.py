#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""半挂车折叠（jackknife）风险模型与限速保护。

模型是经典的「牵引车 + 挂车」运动学（hitch 在牵引车后轴）：

    牵引车航向      theta1' = v * tan(delta) / L1
    挂车航向        theta2' = (v / L2) * sin(theta1 - theta2)
    铰接角          phi = theta2 - theta1

对 ``phi`` 求导得到

    phi' = -(v / L2) * sin(phi) - (v * tan(delta)) / L1

前进（v > 0）时 ``d(phi')/d(phi) = -(v/L2) < 0``，铰接角自稳；
倒车（v < 0）时该导数为正，铰接角会指数发散——这就是倒车容易折叠的原因。

本模块给出：铰接角速率、折叠时间预测、给定转角下的最大安全倒车速度、风险分级。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

SAFE_ANGLE = math.radians(40.0)
CAUTION_ANGLE = math.radians(60.0)
JACKKNIFE_ANGLE = math.radians(75.0)


@dataclass
class TrailerParams:
    tractor_wheelbase: float = 3.6      # L1 [m]
    trailer_wheelbase: float = 8.0      # L2，hitch 到挂车轴 [m]
    max_steer: float = math.radians(35.0)
    jackknife_angle: float = JACKKNIFE_ANGLE

    def __post_init__(self):
        if self.tractor_wheelbase <= 0 or self.trailer_wheelbase <= 0:
            raise ValueError("轴距必须 > 0")
        if not 0.0 < self.max_steer < math.pi / 2:
            raise ValueError("最大转角必须在 (0, pi/2)")
        if not 0.0 < self.jackknife_angle < math.pi / 2:
            raise ValueError("折叠角必须在 (0, pi/2)")


def hitch_rate(v: float, delta: float, phi: float, p: TrailerParams) -> float:
    """铰接角变化率 d(phi)/dt [rad/s]。"""
    return -(v / p.trailer_wheelbase) * math.sin(phi) \
        - (v * math.tan(delta)) / p.tractor_wheelbase


def risk_level(phi: float, p: TrailerParams | None = None) -> int:
    """0 安全 / 1 注意 / 2 危险 / 3 折叠。"""
    p = p or TrailerParams()
    a = abs(phi)
    if a >= p.jackknife_angle:
        return 3
    if a >= CAUTION_ANGLE:
        return 2
    if a >= SAFE_ANGLE:
        return 1
    return 0


def time_to_jackknife(v: float, delta: float, phi0: float,
                      p: TrailerParams, dt: float = 0.01,
                      max_time: float = 120.0,
                      angle: float | None = None) -> float:
    """从当前铰接角出发，按恒定 v、delta 积分到折叠角所需时间 [s]。

    已经超过折叠角返回 0；在 ``max_time`` 内不会折叠返回 ``inf``。
    """
    limit = p.jackknife_angle if angle is None else float(angle)
    phi = float(phi0)
    if abs(phi) >= limit:
        return 0.0
    steps = int(round(max_time / dt))
    for k in range(steps):
        phi += hitch_rate(v, delta, phi, p) * dt
        if abs(phi) >= limit:
            return (k + 1) * dt
    return float("inf")


def max_safe_reverse_speed(phi0: float, delta: float, p: TrailerParams,
                           horizon: float = 5.0,
                           dt: float = 0.01,
                           margin: float = 0.85) -> float:
    """在 ``horizon`` 秒内不折叠的前提下，允许的最大倒车速度 [m/s]（正数）。"""
    limit = p.jackknife_angle * margin
    if abs(phi0) >= limit:
        return 0.0
    lo, hi = 0.0, 8.0
    # 速度为 0 一定安全；hi 通常不安全，二分找边界
    if time_to_jackknife(-hi, delta, phi0, p, dt, max_time=horizon * 1.5, angle=limit) > horizon:
        return hi
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if time_to_jackknife(-mid, delta, phi0, p, dt, max_time=horizon * 1.5, angle=limit) > horizon:
            lo = mid
        else:
            hi = mid
    return lo


def guard_state(phi: float, delta: float, requested_speed: float,
                p: TrailerParams, horizon: float = 5.0):
    """返回 (允许速度, 风险等级, 说明)。requested_speed 为倒车速度（m/s，正数）。"""
    level = risk_level(phi, p)
    allowed = max_safe_reverse_speed(phi, delta, p, horizon)
    if level >= 3:
        return 0.0, level, "jackknife: 已超过折叠角，禁止继续倒车"
    if abs(requested_speed) > allowed:
        return allowed, max(level, 2), (
            "限速: 请求 %.2f m/s 超过允许 %.2f m/s（铰接角 %.1f°，转角 %.1f°）"
            % (abs(requested_speed), allowed, math.degrees(phi),
               math.degrees(delta)))
    return allowed, level, (
        "允许倒车 %.2f m/s（铰接角 %.1f°，转角 %.1f°）"
        % (allowed, math.degrees(phi), math.degrees(delta)))
