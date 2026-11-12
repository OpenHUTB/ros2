#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机控制器：朝目标前沿点偏航跟踪，带速度阻尼与近点悬停。"""

from __future__ import annotations

import math

V_MAX = 0.80
YAW_MAX = 1.40
KP_YAW = 2.20
VEL_DAMPING = 2.00
MIN_FORWARD = 0.15
REACH_DIST = 0.10


class ExplorerController:
    """二维位姿控制器。"""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = 0.0
        self.vy = 0.0

    def heading_error(self, tx: float, ty: float) -> float:
        desired = math.atan2(ty - self.y, tx - self.x)
        err = desired - self.yaw
        return math.atan2(math.sin(err), math.cos(err))

    def update(self, tx: float, ty: float, dt: float,
               clearance: dict | None = None) -> dict:
        clearance = clearance or {"ahead": True, "left": True, "right": True}
        dist = math.hypot(tx - self.x, ty - self.y)
        if dist <= REACH_DIST:
            self.vx = 0.0
            self.vy = 0.0
            return {"reached": True, "dist": dist, "speed": 0.0, "yaw_cmd": 0.0}

        err = self.heading_error(tx, ty)
        yaw_cmd = max(-YAW_MAX, min(YAW_MAX, KP_YAW * err))

        if not clearance["ahead"]:
            # 前方受阻：朝自由侧转向，低速绕行。
            if clearance["left"] and not clearance["right"]:
                yaw_cmd = YAW_MAX
            elif clearance["right"] and not clearance["left"]:
                yaw_cmd = -YAW_MAX
            forward = 0.0
        else:
            forward = V_MAX * max(0.0, math.cos(err))
        forward = max(0.0, forward)

        target_vx = forward * math.cos(self.yaw)
        target_vy = forward * math.sin(self.yaw)
        self.vx += (target_vx - self.vx) * min(1.0, VEL_DAMPING * dt)
        self.vy += (target_vy - self.vy) * min(1.0, VEL_DAMPING * dt)

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw += yaw_cmd * dt

        speed = math.hypot(self.vx, self.vy)
        return {"reached": False, "dist": dist, "speed": speed, "yaw_cmd": yaw_cmd}
