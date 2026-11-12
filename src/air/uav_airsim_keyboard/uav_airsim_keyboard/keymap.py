#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯逻辑模块：按键 -> Twist 分量的映射与合成。

这里刻意不 import rclpy，方便在没有 ROS 的机器（以及 CI）里直接跑单元测试。
"""

import math

# 按键 -> (linear.x, linear.y, linear.z, angular.z) 的方向系数
KEY_DIRECTION = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),   # 左移：linear.y 左为正
    'd': (0.0, -1.0, 0.0, 0.0),  # 右移
    'r': (0.0, 0.0, 1.0, 0.0),
    'f': (0.0, 0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
}



def combine_keys(active, speed, yaw_rate):
    """把当前按住的键合成一条 Twist 分量。

    同时按住两个方向键时，分量相加会让合速度变成 sqrt(2)*speed，
    这里按合速度等比缩放回 speed，保证斜向与直行的速度一致。
    """
    vx = vy = vz = wz = 0.0
    for key in active:
        dx, dy, dz, dyaw = KEY_DIRECTION[key]
        vx += dx * speed
        vy += dy * speed
        vz += dz * speed
        wz += dyaw * yaw_rate
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if norm > speed > 0.0:
        scale = speed / norm
        vx *= scale
        vy *= scale
        vz *= scale
    return vx, vy, vz, wz
