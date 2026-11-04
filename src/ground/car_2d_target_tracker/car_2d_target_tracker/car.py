#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单轮车（unicycle）模型：只做运动学，不含渲染。

    x' = x + v·cos(θ)·dt
    y' = y + v·sin(θ)·dt
    θ' = θ + ω·dt

与原实现的差别只有一处（但很关键）：**整段扫掠检查**而不是只看新位置。
原实现用终点判断碰撞，高速或大步长时会一步跨过障碍物（穿模）；这里沿线段采样，
并且撞上时把车"贴"在障碍表面而不是直接丢弃这一步（否则会出现明显的位置跳变）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from car_2d_target_tracker.geometry import sweep_hits, collides, dist


@dataclass
class CarState:
    x: float
    y: float
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    trail: list = field(default_factory=list)


class Car:
    """运动学小车。``step`` 接收期望 (v, omega)，返回是否被障碍挡住。"""

    def __init__(self, x, y, theta=0.0, radius=10.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = 0.0
        self.omega = 0.0
        self.radius = float(radius)
        self.trail = [(self.x, self.y)]
        self.blocked_steps = 0

    # ---------------------------------------------------------------- 状态
    @property
    def state(self):
        return CarState(self.x, self.y, self.theta, self.v, self.omega, list(self.trail))

    def reset(self, x, y, theta=0.0):
        self.x, self.y, self.theta = float(x), float(y), float(theta)
        self.v = self.omega = 0.0
        self.trail = [(self.x, self.y)]
        self.blocked_steps = 0

    # ---------------------------------------------------------------- 步进
    def step(self, v, omega, dt, obstacles=()):
        """按 (v, omega) 推进一步；撞到障碍时只保留不撞的那部分位移。

        返回 True 表示这一步被障碍挡住（调用方据此触发绕障逻辑）。
        """
        v, omega = float(v), float(omega)
        new_theta = self.theta + omega * dt
        new_x = self.x + v * math.cos(new_theta) * dt
        new_y = self.y + v * math.sin(new_theta) * dt

        self.v, self.omega = v, omega
        if not obstacles or not sweep_hits(self.x, self.y, new_x, new_y,
                                           self.radius, obstacles):
            self.x, self.y, self.theta = new_x, new_y, new_theta
            self.trail.append((self.x, self.y))
            return False

        # 被挡住：沿路径回退到"第一个碰撞点之前"，让车贴住障碍而不是原地不动
        from car_2d_target_tracker.geometry import block_fraction
        frac = block_fraction(self.x, self.y, new_x, new_y, self.radius, obstacles)
        self.x = self.x + (new_x - self.x) * frac
        self.y = self.y + (new_y - self.y) * frac
        self.theta = new_theta
        self.v = 0.0
        self.blocked_steps += 1
        self.trail.append((self.x, self.y))
        return True

    # ---------------------------------------------------------------- 工具
    def distance_to(self, x, y):
        return dist(self.x, self.y, x, y)
