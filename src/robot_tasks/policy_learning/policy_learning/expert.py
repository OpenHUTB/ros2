"""专家策略：在到目标距离图的下降路径上取前视点，转成速度指令。

网络学的是“前视点在机体坐标系里的位置”（平滑、良定义），
速度指令由 steer() 这条固定控制律换算，避免直接回归角速度的抖动。
"""
from __future__ import annotations

import math

import numpy as np

LOOKAHEAD = 0.8        # 前视距离（米）
V_MAX = 0.8            # 最大线速度（米/秒）
V_MIN = 0.15           # 转弯时保底前进速度
W_MAX = 1.5            # 最大角速度（弧度/秒）
K_ANG = 2.0            # 航向误差 → 角速度

__all__ = ["LOOKAHEAD", "V_MAX", "W_MAX", "target_point", "steer",
           "pure_pursuit"]


def target_point(path, pose):
    """返回前视点在机体坐标系下的 (前向, 左向) 偏移（米）。"""
    x, y, th = pose

    nearest = 0
    best = float("inf")
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - x, py - y)
        if d < best:
            best, nearest = d, i

    target = path[-1]
    for px, py in path[nearest:]:
        if math.hypot(px - x, py - y) >= LOOKAHEAD:
            target = (px, py)
            break

    dx, dy = target[0] - x, target[1] - y
    c, s = math.cos(th), math.sin(th)
    return dx * c + dy * s, -dx * s + dy * c


def steer(fwd: float, lat: float):
    """把机体坐标系下的前视点转成 (v, ω)。"""
    ang = math.atan2(lat, max(fwd, 1e-3))
    v = V_MAX * max(0.0, 1.0 - abs(ang) / (math.pi * 0.5))
    v = max(V_MIN, v)
    w = float(np.clip(K_ANG * ang, -W_MAX, W_MAX))
    return v, w


def pure_pursuit(path, pose):
    fwd, lat = target_point(path, pose)
    return steer(fwd, lat)
