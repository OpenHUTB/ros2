# Copyright 2026 OpenHUTB
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See http://www.apache.org/licenses/LICENSE-2.0 for the full text.
"""栅格地图与几何工具。

本模块不依赖 ROS，全部为纯 numpy 计算，方便单独测试：
四元数转旋转矩阵、世界坐标 <-> 栅格坐标、针孔模型反投影、Bresenham 射线。
"""

import math

import numpy as np


def quat_to_rot_matrix(x, y, z, w):
    """四元数 (x, y, z, w) 转 3x3 旋转矩阵。

    返回 R 满足 p_world = R @ p_body + t。对非单位四元数先按模长归一化，
    避免 AirSim 输出的浮点累积误差放大。
    """
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / norm
    return np.array(
        [
            [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_quat(x, y, z, w):
    """从四元数提取偏航角（弧度，绕世界 z 轴）。"""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def wrap_angle(angle):
    """把角度归一化到 (-pi, pi]，用于航向误差计算。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    """把 value 截断到 [low, high]。"""
    return max(low, min(high, value))


def world_to_grid(x, y, origin_x, origin_y, resolution):
    """世界坐标 -> 栅格索引，用 floor 保证负坐标不再向 0 取整。

    这是原实现用 int() 截断导致“偏半格”的修正点：int(-0.4) == 0，
    而 floor(-0.4) == -1，后者才是正确的格号。
    """
    return (
        int(math.floor((x - origin_x) / resolution)),
        int(math.floor((y - origin_y) / resolution)),
    )


def grid_to_world(gx, gy, origin_x, origin_y, resolution):
    """栅格索引 -> 格子中心的世界坐标。"""
    return (
        origin_x + (gx + 0.5) * resolution,
        origin_y + (gy + 0.5) * resolution,
    )


def camera_intrinsics(width, height, hfov_deg):
    """由水平视场角与图像尺寸算针孔相机内参 (fx, fy, cx, cy)。

    AirSim 默认相机水平 FOV 为 90 度，方形像素下 fy == fx。
    cx/cy 取像素中心减 0.5，与 OpenCV 的像素坐标系约定一致。
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx
    return fx, fy, width / 2.0 - 0.5, height / 2.0 - 0.5


def backproject(u, v, depth, fx, fy, cx, cy):
    """针孔模型反投影：像素 + 深度 -> 相机系三维点。

    输入 u/v/depth 为一维等长数组，输出 3xN 的相机系坐标（单位：米）。
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth
    """
    z = np.asarray(depth, dtype=np.float64)
    x = (np.asarray(u, dtype=np.float64) - cx) * z / fx
    y = (np.asarray(v, dtype=np.float64) - cy) * z / fy
    return np.stack([x, y, z])


def bresenham(gx0, gy0, gx1, gy1):
    """整数栅格上的 Bresenham 直线，返回途经的全部栅格（含端点）。

    建图时用它把“传感器原点 -> 深度端点”这条射线上的格子标记为自由，
    只把最后一个格子标记为占据，实现射线追踪式的占据栅格更新。
    """
    dx = abs(gx1 - gx0)
    dy = abs(gy1 - gy0)
    step_x = 1 if gx0 < gx1 else -1
    step_y = 1 if gy0 < gy1 else -1
    err = dx - dy
    cells = []
    x, y = gx0, gy0
    while True:
        cells.append((x, y))
        if x == gx1 and y == gy1:
            break
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x += step_x
        if err2 < dx:
            err += dx
            y += step_y
    return cells
