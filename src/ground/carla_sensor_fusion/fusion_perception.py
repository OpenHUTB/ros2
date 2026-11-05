# -*- coding: utf-8 -*-
"""fusion_perception.py —— 原始传感器数据 → 目标级观测（纯 numpy，无 CARLA 依赖）

CARLA 版的传感器回调把原始数据（语义分割图 / 点云 / IMU compass）交给本模块，
由这里完成「检测 → 目标级观测」的一步；函数全部是纯函数，便于用合成数据做单元测试
（见 test/test_sensor_fusion.py）。

坐标约定：
  - 相机坐标系：x 前、y 右；方位角右为正。
  - 雷达（LiDAR）坐标系：与本模块一致，x 前、y 左，方位角逆时针为正
    （CARLA LiDAR 点云原生如此）。相机→雷达的轴向差异由外参标定吸收。
"""

import math

import numpy as np

from sensor_fusion_core import rot2, wrap_angle

# CARLA 语义分割：车辆 tag（CityCar / Vehicle 等归入 10）
CARLA_TAG_VEHICLE = 10


def image_to_bearing(u_px, width, fov_deg):
    """图像像素列 u → 相机坐标系方位角（rad，右为正）。

    pinhole 近似：fx = (W/2) / tan(fov/2)，主点在图像中心。
    """
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return math.atan2(float(u_px) - width / 2.0, fx)


def mask_to_bearing(mask, fov_deg, target_tag=CARLA_TAG_VEHICLE, min_pixels=30):
    """语义分割掩码（H×W，值为 CARLA 语义标签）里目标类像素 → 方位观测。

    取目标类像素的质心列反投影；像素数不足 min_pixels 视为未检出。
    返回 (bearing | None, n_pixels)。
    """
    if mask is None or np.size(mask) == 0:
        return None, 0
    mask = np.asarray(mask)
    ys, xs = np.nonzero(mask == target_tag)
    n = int(xs.size)
    if n < min_pixels:
        return None, n
    return image_to_bearing(float(xs.mean()), mask.shape[1], fov_deg), n


def lidar_points_to_observation(points, min_range=1.5, max_range=40.0,
                                z_min=-0.8, z_max=2.5, n_nearest=12):
    """LiDAR 点云 (N,4)[x,y,z,intensity]（雷达坐标系）→ 最近目标观测 (r, θ)。

    过滤距离与高度（去掉地面回波与车身自反射），取最近 n_nearest 个点的均值
    作为目标观测。返回 None 表示本帧未检出。
    """
    if points is None or len(points) == 0:
        return None
    pts = np.asarray(points, dtype=float)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.hypot(x, y)
    keep = (r >= min_range) & (r <= max_range) & (z >= z_min) & (z <= z_max)
    if not np.any(keep):
        return None
    r, y, x = r[keep], y[keep], x[keep]
    th = np.arctan2(y, x)
    k = np.argsort(r)[:min(n_nearest, r.size)]
    return float(r[k].mean()), float(np.mean(th[k]))


def imu_compass_to_yaw(compass_rad):
    """CARLA IMU compass（自北向顺时针，rad）→ Unreal yaw（自 +X 顺时针，rad）。

    compass=0 指北(+Y) 对应 yaw=90°，故 yaw = π/2 - compass。
    """
    return wrap_angle(math.pi / 2.0 - float(compass_rad))


def transform_camera_bearing(theta_cam, r_ref, extrinsic):
    """用标定外参把相机坐标系方位观测对齐到雷达坐标系方位。

    p_cam = r_ref·[cos θ_cam, sin θ_cam]；p_radar = R(Δθ)·p_cam + t，
    θ_radar = atan2(p_radar_y, p_radar_x)。

    extrinsic = (theta_deg, tx, ty)，与 estimate_extrinsic_2d 的返回前三项一致；
    相机只有方位、没有距离，r_ref 取同帧雷达距离或 EKF 预测距离作为近似。
    """
    th_deg, tx, ty = extrinsic[0], extrinsic[1], extrinsic[2]
    p_cam = np.array([r_ref * math.cos(theta_cam), r_ref * math.sin(theta_cam)])
    p_rad = rot2(math.radians(th_deg)) @ p_cam + np.array([tx, ty])
    return math.atan2(float(p_rad[1]), float(p_rad[0]))


def points_to_extrinsic_pairs(camera_bearings, lidar_obs):
    """把同一时刻的 (相机方位, 雷达 r/θ) 配对成标定点，用于在线外参标定。

    相机缺距离，用该时刻的雷达距离作为两点共同的距离基准：
      p_cam   = r·[cos θ_cam,  sin θ_cam]
      p_lidar = r·[cos θ_lidar, sin θ_lidar]
    返回 (pts_cam, pts_lidar) 两个 (N,2) 数组。
    """
    pts_cam, pts_lidar = [], []
    for th_cam, obs in zip(camera_bearings, lidar_obs):
        if th_cam is None or obs is None:
            continue
        r, th_l = obs
        pts_cam.append([r * math.cos(th_cam), r * math.sin(th_cam)])
        pts_lidar.append([r * math.cos(th_l), r * math.sin(th_l)])
    if not pts_cam:
        return np.zeros((0, 2)), np.zeros((0, 2))
    return np.array(pts_cam), np.array(pts_lidar)


def radar_points_to_observation(points, min_range=1.5, max_range=40.0,
                                n_nearest=6):
    """CARLA Radar 原始检测点 (N,4)[depth, azimuth, altitude, velocity]
    → 最近目标观测 (r, θ)。

    CARLA sensor.other.radar 每点 4 个 float32：斜距 depth(m)、水平方位
    azimuth(rad，以传感器 forward 为 0)、俯仰 altitude(rad)、径向速度
    velocity(m/s)。按 depth 过滤后取最近若干点均值作为目标观测；
    方位与 LiDAR/相机同为 ego 体坐标（x 前、y 右，右为正），可直接配对标定。
    返回 None 表示本帧未检出。
    """
    if points is None or len(points) == 0:
        return None
    pts = np.asarray(points, dtype=float)
    depth, azimuth = pts[:, 0], pts[:, 1]
    keep = (depth >= min_range) & (depth <= max_range)
    if not np.any(keep):
        return None
    depth, azimuth = depth[keep], azimuth[keep]
    k = np.argsort(depth)[:min(n_nearest, depth.size)]
    return float(depth[k].mean()), float(np.mean(azimuth[k]))

