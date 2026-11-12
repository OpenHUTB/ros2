#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动控制器：将端到端策略输出的前飞与偏航指令积分到机体位姿。"""

from __future__ import annotations

import math

V_MAX = 0.90
YAW_MAX = 1.20


class ImagePolicyController:
    """二维位姿积分器。"""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def update(self, forward: float, yaw_cmd: float, dt: float) -> dict:
        self.yaw += max(-YAW_MAX, min(YAW_MAX, yaw_cmd)) * dt
        forward = max(0.0, min(V_MAX, forward))
        self.x += forward * math.cos(self.yaw) * dt
        self.y += forward * math.sin(self.yaw) * dt
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "forward": forward}
