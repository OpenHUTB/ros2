#!/usr/bin/env python3
"""lidar_utils.py —— LiDAR 点云预处理与扇区特征提取

感知层：将 AirSim 返回的 3D 点云按方位角划分为 N 个扇区，
每个扇区取最小水平距离作为特征值，得到低维距离特征向量。

扇区特征公式：
    d_i = min_{j in S_i} || p_j^{xy} ||_2 ,  i = 1 .. N
其中 S_i 为第 i 个扇区覆盖的角度区间：
    S_i = [ -pi + (i-1)*2pi/N,  -pi + i*2pi/N )
"""
import numpy as np


def lidar_points_to_numpy(lidar_data, max_range=10.0):
    """将 AirSim lidar 返回对象转为 Nx3 点云数组。

    Args:
        lidar_data: airsim.LidarData，含 point_cloud(flatten 的 x,y,z,反射强度...)
        max_range: 最大量程，越界点剔除

    Returns:
        np.ndarray (N,3) 或 None（无数据时）
    """
    if lidar_data is None:
        return None
    pc = getattr(lidar_data, "point_cloud", None)
    if pc is None or len(pc) < 3:
        return None
    arr = np.array(pc, dtype=np.float64)
    n = len(arr)
    # 兼容两种点云格式：纯 (x,y,z) 或 (x,y,z,intensity)
    if n % 4 == 0:
        arr = arr.reshape(-1, 4)[:, :3]     # 含强度格式
    else:
        arr = arr.reshape(-1, 3)            # 纯 xyz 格式
    dist = np.linalg.norm(arr[:, :2], axis=1)
    arr = arr[(dist > 0.05) & (dist <= max_range)]
    return arr if len(arr) > 0 else None


def points_to_sector_features(points, num_sectors=16, max_range=10.0):
    """点云 -> 扇区最小距离特征向量。

    Args:
        points: (N,3) 点云（x,y,z，z 为高度，忽略）
        num_sectors: 扇区数量 N
        max_range: 无数据扇区填充值（默认量程）

    Returns:
        np.ndarray (N,) 扇区距离特征，值域 [0, max_range]
    """
    features = np.full(num_sectors, float(max_range))
    if points is None or len(points) == 0:
        return features

    xy = points[:, :2]
    angles = np.arctan2(xy[:, 1], xy[:, 0])            # [-pi, pi]
    dists = np.linalg.norm(xy, axis=1)

    # 将角度映射到扇区编号 [0, num_sectors)
    idx = np.floor((angles + np.pi) / (2.0 * np.pi) * num_sectors).astype(np.int64)
    idx = np.clip(idx, 0, num_sectors - 1)

    for i in range(num_sectors):
        mask = idx == i
        if np.any(mask):
            features[i] = np.min(dists[mask])
    return features


def normalize_features(features, sector_max=8.0):
    """特征归一化到 [0,1]，便于神经网络输入。"""
    return np.clip(features, 0.0, sector_max) / sector_max


def goal_direction_feature(yaw, goal_xy):
    """目标方向特征：机体坐标系下的目标方向 (sin, cos)。

    将全局目标位置转换到机体坐标：
        dx' = cos(yaw)*dx + sin(yaw)*dy
        dy' = -sin(yaw)*dx + cos(yaw)*dy
    返回 (sin(theta), cos(theta))，theta 为机体前方为 0 的目标方位角。

    Args:
        yaw: 当前偏航角(rad)
        goal_xy: 目标在机体坐标系下的相对坐标 (dx, dy)

    Returns:
        np.ndarray (2,)
    """
    dx, dy = float(goal_xy[0]), float(goal_xy[1])
    local_dx = np.cos(yaw) * dx + np.sin(yaw) * dy
    local_dy = -np.sin(yaw) * dx + np.cos(yaw) * dy
    norm = np.hypot(local_dx, local_dy) + 1e-6
    return np.array([local_dx / norm, local_dy / norm], dtype=np.float64)
