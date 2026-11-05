#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目标运动：匀速直线 + 边界反弹，或静止。

用 ``random.Random(seed)`` 而不是全局 random：同一 seed 必须能复现同一段轨迹，
否则"到达时间"这类指标每次跑都不一样，没法写进文档、也没法写测试。
"""
from __future__ import annotations

import math
import random


class Target:
    def __init__(self, x, y, speed=60.0, width=900.0, height=600.0, margin=25.0,
                 motion="wander", seed=42):
        self.x = float(x)
        self.y = float(y)
        self.speed = float(speed)
        self.width = float(width)
        self.height = float(height)
        self.margin = float(margin)
        self.motion = motion
        rng = random.Random(seed)
        angle = rng.uniform(0.0, 2.0 * math.pi)
        self.vx = self.speed * math.cos(angle) if motion == "wander" else 0.0
        self.vy = self.speed * math.sin(angle) if motion == "wander" else 0.0

    def step(self, dt):
        if self.motion != "wander":
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        m = self.margin
        if self.x < m or self.x > self.width - m:
            self.vx *= -1.0
            self.x = max(m, min(self.width - m, self.x))
        if self.y < m or self.y > self.height - m:
            self.vy *= -1.0
            self.y = max(m, min(self.height - m, self.y))

    def set_pos(self, x, y):
        self.x, self.y = float(x), float(y)
