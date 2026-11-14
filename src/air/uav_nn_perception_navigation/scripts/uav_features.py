#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达特征提取（训练与部署共用）

把 AirSim 返回的点云压成一个固定长度的向量，供神经网络使用：
    前 12 维：把 ±30° 视场均分成 12 个扇区，每扇区取最近点距离，按量程归一化
    后 4 维：目标相对无人机的位置，转到机体系后的 [距离, sin 方位角, cos 方位角, 高度差]
"""

import math

import numpy as np

NUM_SECTORS = 12
FOV_DEG = 60.0
MAX_RANGE = 40.0
MAX_GOAL_DIST = 20.0
MAX_DZ = 10.0
FEATURE_DIM = NUM_SECTORS + 4


def scan_angles(fov_deg, bins):
    """Angles for equally spaced LaserScan bins, including the last sample."""
    if bins <= 0 or not 0 < fov_deg <= 360:
        raise ValueError('bins must be positive and field of view in (0, 360]')
    first = math.radians(-fov_deg / 2.0)
    increment = math.radians(fov_deg / bins)
    return first, first + (bins - 1) * increment, increment


def extract_sector_features(points, num_sectors=NUM_SECTORS, fov_deg=FOV_DEG,
                            max_range=MAX_RANGE):
    """点云 -> 每扇区最近距离（归一化到 0~1，1 表示该方向没有障碍或超出量程）。

    points: Nx3 的数组，坐标系为传感器系（x 前、y 右、z 下）。
    """
    feats = np.ones(num_sectors, dtype=np.float32)
    if points is None or len(points) == 0:
        return feats

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    x = pts[:, 0]
    y = pts[:, 1]
    # 只保留前方、且落在量程内的点
    r = np.sqrt(x * x + y * y)
    ang = np.degrees(np.arctan2(y, x))
    valid = (r > 0.05) & (r < max_range) & (np.abs(ang) <= fov_deg / 2.0)
    if not np.any(valid):
        return feats

    r = r[valid]
    ang = ang[valid]
    half = fov_deg / 2.0
    idx = np.clip(((ang + half) / fov_deg * num_sectors).astype(np.int32),
                  0, num_sectors - 1)

    for sector in range(num_sectors):
        sel = r[idx == sector]
        if sel.size:
            feats[sector] = min(1.0, float(sel.min()) / max_range)
    return feats


def scan_angles(fov_deg, bins):
    """LaserScan 首末采样角度与间隔。"""
    if bins <= 0 or not 0 < fov_deg <= 360:
        raise ValueError('bins must be positive and fov_deg in (0, 360]')
    first = math.radians(-fov_deg / 2.0)
    increment = math.radians(fov_deg / bins)
    return first, first + (bins - 1) * increment, increment


def extract_scan(points, bins=180, fov_deg=180.0, max_range=MAX_RANGE):
    """把点云投影成 2D 环形扫描（给 SLAM 用，角度遵循 ROS 约定）。

    AirSim 传感器系是 x 前、y 右（NED），`atan2(y, x)` 的正角在右侧；
    ROS LaserScan（REP 103）的 0 是正前方、**逆时针（向左）为正**。
    直接写进 LaserScan 会让地图左右镜像，所以这里先取负。

    返回长度为 bins 的距离数组，角度从 -fov/2 到 +fov/2 均匀分布；
    没有命中点的角度填 max_range。
    """
    ranges = np.full(bins, max_range, dtype=np.float32)
    if points is None or len(points) == 0:
        return ranges

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    x, y = pts[:, 0], pts[:, 1]
    r = np.sqrt(x * x + y * y)
    # 取负：AirSim 右正 -> ROS 左正，否则 LaserScan 左右镜像
    ang = -np.degrees(np.arctan2(y, x))
    half = fov_deg / 2.0
    valid = (r > 0.1) & (r < max_range) & (np.abs(ang) <= half)
    if not np.any(valid):
        return ranges

    r, ang = r[valid], ang[valid]
    idx = np.clip(((ang + half) / fov_deg * bins).astype(np.int32), 0, bins - 1)
    for i in range(bins):
        sel = r[idx == i]
        if sel.size:
            ranges[i] = float(sel.min())
    return ranges


def _rotation_matrix(orientation):
    """机体系到 NED 世界系的旋转矩阵；兼容旧代码的 yaw 标量。"""
    if np.isscalar(orientation):
        half = float(orientation) / 2.0
        x, y, z, w = 0.0, 0.0, math.sin(half), math.cos(half)
    elif hasattr(orientation, 'x_val'):
        x, y, z, w = (orientation.x_val, orientation.y_val,
                      orientation.z_val, orientation.w_val)
    else:
        x, y, z, w = orientation
    q = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError('orientation quaternion must be nonzero')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def transform_points_to_frame(points, src_pos, src_orientation,
                              dst_pos, dst_orientation, sensor_offset=(0, 0, 0)):
    """把 src 位姿下的机体系点云转到 dst 位姿的机体系（NED，完整三轴旋转）。

    多帧累积建图时必须先做这一步：雷达每帧只吐一小段，拼接时如果直接用
    原始坐标，无人机一旦平移或转向，旧点就会留在错误的方位上，地图会拖影。
    """
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if len(pts) == 0:
        return pts

    src_rot = _rotation_matrix(src_orientation)
    dst_rot = _rotation_matrix(dst_orientation)
    offset = np.asarray(sensor_offset, dtype=np.float64)
    world = (pts + offset) @ src_rot.T + np.asarray(src_pos, dtype=np.float64)
    return ((world - np.asarray(dst_pos, dtype=np.float64)) @ dst_rot - offset).astype(np.float32)


def goal_features(current_pos, goal_pos, yaw_ned):
    """目标相对量：距离、方位角的正余弦、高度差，均已归一化。

    current_pos / goal_pos 为 NED 坐标 (x, y, z)，yaw_ned 为弧度。
    """
    dx = float(goal_pos[0] - current_pos[0])
    dy = float(goal_pos[1] - current_pos[1])
    dz = float(goal_pos[2] - current_pos[2])

    dist = math.sqrt(dx * dx + dy * dy)
    bearing = math.atan2(dy, dx) - yaw_ned       # 转到机体系
    bearing = math.atan2(math.sin(bearing), math.cos(bearing))

    return np.array([
        min(1.0, dist / MAX_GOAL_DIST),
        math.sin(bearing),
        math.cos(bearing),
        max(-1.0, min(1.0, dz / MAX_DZ)),
    ], dtype=np.float32)


def build_feature_vector(points, current_pos, goal_pos, yaw_ned):
    return np.concatenate([
        extract_sector_features(points),
        goal_features(current_pos, goal_pos, yaw_ned),
    ]).astype(np.float32)


def yaw_from_quaternion(q):
    """AirSim 四元数（NED）-> 偏航角弧度。"""
    x, y, z, w = q.x_val, q.y_val, q.z_val, q.w_val
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
