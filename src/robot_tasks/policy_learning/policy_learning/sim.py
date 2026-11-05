"""差速底盘运动学与碰撞检测。"""
from __future__ import annotations

import math

import numpy as np

from .grid import RES, H, W

DT = 0.1            # 控制周期（秒）
ROBOT_R = 0.18      # 机体半径（米），用于碰撞判定

__all__ = ["DT", "ROBOT_R", "step", "collides"]


def step(pose, v: float, w: float, dt: float = DT):
    """差速底盘积分：先按平均航向位移，再更新航向。"""
    x, y, th = pose
    th2 = th + w * dt
    x2 = x + v * math.cos((th + th2) * 0.5) * dt
    y2 = y + v * math.sin((th + th2) * 0.5) * dt
    return x2, y2, th2


def collides(g: np.ndarray, pose, radius: float = ROBOT_R) -> bool:
    """机体圆是否与任一占据格相交（检查圆周 8 点 + 中心）。"""
    x, y, _ = pose
    i = int(np.floor(x / RES))
    j = int(np.floor(y / RES))
    if not (0 <= i < W and 0 <= j < H) or g[j, i]:
        return True
    for k in range(8):
        a = k * math.pi / 4.0
        ci = int(np.floor((x + radius * math.cos(a)) / RES))
        cj = int(np.floor((y + radius * math.sin(a)) / RES))
        if not (0 <= ci < W and 0 <= cj < H):
            return True
        if g[cj, ci]:
            return True
    return False
