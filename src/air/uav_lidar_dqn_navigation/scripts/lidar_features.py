#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达特征工程（DQN 训练与部署共用）

AirSim 的 lidar_1 水平视场为 ±90°（180°），16 线，量程 40 m。
本模块把点云压成固定长度状态向量：

  24 维  把 180° 视场等分成 24 个扇区，每扇区取最近距离 / 量程（1 表示通畅）
   4 维  目标相对量 [水平距离, sin(方位), cos(方位), 高度差]，均已归一化
   3 维  机体系速度 [vx, vy, vz] / 速度上限
   1 维  上一步动作编号（归一化），让策略知道刚才做了什么

合计 32 维，对应 DQN 的输入。建图用的扫描另外由 extract_scan() 生成。
"""

import math

import numpy as np

NUM_SECTORS = 24
FOV_DEG = 180.0
MAX_RANGE = 40.0
MAX_GOAL_DIST = 40.0
MAX_DZ = 8.0
MAX_SPEED = 5.0
NUM_ACTIONS = 6
STATE_DIM = NUM_SECTORS + 4 + 3 + 1


def sector_distances(points, num_sectors=NUM_SECTORS, fov_deg=FOV_DEG,
                     max_range=MAX_RANGE):
    """点云 -> 各扇区最近距离（归一化，1 表示该方向通畅）。

    points 为传感器系 Nx3（x 前、y 右、z 下），AirSim 已用
    DataFrame=SensorLocalFrame 输出，无需再做坐标变换。
    """
    dists = np.ones(num_sectors, dtype=np.float32)
    if points is None:
        return dists
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if pts.shape[0] == 0:
        return dists

    x, y = pts[:, 0], pts[:, 1]
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))
    half = fov_deg / 2.0
    keep = (r > 0.15) & (r < max_range) & (np.abs(ang) <= half)
    if not np.any(keep):
        return dists

    r, ang = r[keep], ang[keep]
    idx = np.clip(((ang + half) / fov_deg * num_sectors).astype(np.int32),
                  0, num_sectors - 1)
    for s in range(num_sectors):
        hit = r[idx == s]
        if hit.size:
            dists[s] = min(1.0, float(hit.min()) / max_range)
    return dists


def goal_state(cur_ned, goal_ned, yaw_ned):
    """目标相对量：水平距离、机体系方位角正余弦、高度差。"""
    dx = float(goal_ned[0] - cur_ned[0])
    dy = float(goal_ned[1] - cur_ned[1])
    dz = float(goal_ned[2] - cur_ned[2])

    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) - yaw_ned
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))
    return np.array([
        min(1.0, dist / MAX_GOAL_DIST),
        math.sin(bearing),
        math.cos(bearing),
        max(-1.0, min(1.0, dz / MAX_DZ)),
    ], dtype=np.float32)


def velocity_state(vel_ned, yaw_ned):
    """把 NED 速度旋到机体系再归一化。"""
    vx, vy, vz = float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])
    c, s = math.cos(yaw_ned), math.sin(yaw_ned)
    vx_body = vx * c + vy * s
    vy_body = -vx * s + vy * c
    return np.array([
        max(-1.0, min(1.0, vx_body / MAX_SPEED)),
        max(-1.0, min(1.0, vy_body / MAX_SPEED)),
        max(-1.0, min(1.0, vz / MAX_SPEED)),
    ], dtype=np.float32)


def build_state(points, cur_ned, goal_ned, yaw_ned, vel_ned, last_action=0):
    """拼出 32 维 DQN 状态向量。"""
    return np.concatenate([
        sector_distances(points),
        goal_state(cur_ned, goal_ned, yaw_ned),
        velocity_state(vel_ned, yaw_ned),
        np.array([last_action / float(NUM_ACTIONS)], dtype=np.float32),
    ]).astype(np.float32)


def extract_scan(points, bins=180, fov_deg=FOV_DEG, max_range=MAX_RANGE):
    """点云 -> LaserScan 距离数组，无命中的角度填 max_range。"""
    ranges = np.full(bins, max_range, dtype=np.float32)
    if points is None:
        return ranges
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if pts.shape[0] == 0:
        return ranges

    x, y = pts[:, 0], pts[:, 1]
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))
    half = fov_deg / 2.0
    keep = (r > 0.15) & (r < max_range) & (np.abs(ang) <= half)
    if not np.any(keep):
        return ranges

    r, ang = r[keep], ang[keep]
    idx = np.clip(((ang + half) / fov_deg * bins).astype(np.int32), 0, bins - 1)
    for i in range(bins):
        hit = r[idx == i]
        if hit.size:
            ranges[i] = float(hit.min())
    return ranges


def scan_angle_max(angle_min, angle_increment, bins):
    """LaserScan 的 angle_max：bins 个采样点里最后一个点的角度。

    ROS 的约定是 angle_max = angle_min + (bins - 1) * angle_increment，
    180 个 1° 的点覆盖 [-90°, +89°]，写成 +90° 会让消费者按
    (max - min) / increment + 1 算出 181 个点。
    """
    return angle_min + angle_increment * (bins - 1)


def yaw_from_quaternion(q):
    """AirSim 四元数（NED）-> 偏航角（弧度，顺时针为正）。"""
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def min_obstacle_distance(points, max_range=MAX_RANGE):
    """前向 ±90° 内最近障碍距离（米），无命中返回 max_range。"""
    if points is None:
        return max_range
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if pts.shape[0] == 0:
        return max_range
    x, y = pts[:, 0], pts[:, 1]
    r = np.hypot(x, y)
    ang = np.degrees(np.arctan2(y, x))
    keep = (r > 0.15) & (r < max_range) & (np.abs(ang) <= FOV_DEG / 2.0)
    if not np.any(keep):
        return max_range
    return float(np.min(r[keep]))
