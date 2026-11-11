# -*- coding: utf-8 -*-
"""fusion_perception.py —— 原始传感器数据 → 目标级观测（纯 numpy，无 CARLA 依赖）

CARLA 版的传感器回调把原始数据（语义分割图 / 点云 / IMU compass）交给本模块，
由这里完成「检测 → 目标级观测」；函数全部是纯函数，便于用合成数据做单元测试。

**目标关联（评审点 2/3/4）**：三个检测函数都支持 `pred` 参数（来自 EKF 预测的目标
在 ego 体坐标系下的距离/方位）。给了 `pred` 时按「距离门限 + 方位门限」选目标，
避免道路边缘、路桩、杂波或其他车辆被当成目标；未给 `pred`（首帧 bootstrap）时退回
「最近点/最近簇/全图质心」。LiDAR 使用体素降采样 + 欧式聚类后按簇质心关联。

坐标约定：
  - 相机坐标系：x 前、y 右；方位角右为正。
  - 雷达（LiDAR）坐标系：x 前、y 左，方位角逆时针为正（CARLA LiDAR 原生如此）。
  - CARLA 毫米波雷达 azimuth：右为正（与雷达传感器约定一致，与 LiDAR 相反）。
    相机→雷达的轴向差异由外参标定吸收。
"""

import math

import numpy as np

from sensor_fusion_core import rot2, wrap_angle

# CARLA 语义分割：车辆 tag（CityCar / Vehicle 等归入 10）
CARLA_TAG_VEHICLE = 10


def _xy(r, th):
    """极坐标 (r, θ) → 平面 (x, y)。"""
    return np.array([r * math.cos(th), r * math.sin(th)])


def _polar(xy):
    """平面 (x, y) → 极坐标 (r, θ)。"""
    return float(np.hypot(xy[0], xy[1])), float(math.atan2(xy[1], xy[0]))


def image_to_bearing(u_px, width, fov_deg):
    """图像像素列 u → 相机坐标系方位角（rad，右为正）。

    pinhole 近似：fx = (W/2) / tan(fov/2)，主点在图像中心。
    """
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return math.atan2(float(u_px) - width / 2.0, fx)


def bearing_to_pixel(theta, width, fov_deg):
    """相机方位角（右为正）→ 像素列，用于 ROI 关联。"""
    fx = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return width / 2.0 + fx * math.tan(float(theta))


def mask_to_bearing(mask, fov_deg, target_tag=CARLA_TAG_VEHICLE,
                    min_pixels=30, pred_bearing=None, roi_px=80):
    """语义分割掩码里目标类像素 → 方位观测。

    pred_bearing 给出时（EKF 预测的相机方位），只在对应像素列 ±roi_px 的 ROI 内
    取目标像素质心，实现目标关联，避免其他车辆/误检进入观测（评审点 2）。
    返回 (bearing | None, n_pixels)。
    """
    if mask is None or np.size(mask) == 0:
        return None, 0
    mask = np.asarray(mask)
    ys, xs = np.nonzero(mask == target_tag)
    n = int(xs.size)
    if pred_bearing is not None and n > 0:
        u0 = bearing_to_pixel(pred_bearing, mask.shape[1], fov_deg)
        keep = np.abs(xs - u0) <= roi_px
        xs, ys = xs[keep], ys[keep]
        n = int(xs.size)
    if n < min_pixels:
        return None, n
    return image_to_bearing(float(xs.mean()), mask.shape[1], fov_deg), n


# ---------------------------------------------------------------- 点云聚类
def voxel_downsample(points_xy, voxel=0.4):
    """体素网格降采样，(N,2) → 每体素取均值（与注释一致），降低聚类复杂度。"""
    pts = np.asarray(points_xy, dtype=float)
    if len(pts) == 0:
        return pts
    key = np.floor(pts / voxel).astype(int)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    sums = np.zeros((len(uniq), 2), dtype=float)
    counts = np.zeros(len(uniq), dtype=float)
    np.add.at(sums, inv, pts)
    np.add.at(counts, inv, 1.0)
    return sums / counts[:, None]


def cluster_points_2d(points_xy, eps=1.2, min_pts=3):
    """欧式聚类（半径 eps 连通），返回 [(质心 (x,y), 点数)]。

    在体素降采样后的少量点上做 O(N²) 邻接遍历，保证每个簇有 min_pts 个点。
    """
    n = len(points_xy)
    if n == 0:
        return []
    labels = -np.ones(n, dtype=int)
    cid = 0
    for i in range(n):
        if labels[i] != -1:
            continue
        stack = [i]
        labels[i] = cid
        while stack:
            j = stack.pop()
            d = np.hypot(points_xy[:, 0] - points_xy[j, 0],
                         points_xy[:, 1] - points_xy[j, 1])
            nb = np.where((d <= eps) & (labels == -1))[0]
            for b in nb:
                labels[b] = cid
                stack.append(int(b))
        cid += 1
    clusters = []
    for c in range(cid):
        m = labels == c
        if int(m.sum()) >= min_pts:
            clusters.append((points_xy[m].mean(axis=0), int(m.sum())))
    return clusters


def lidar_points_to_observation(points, min_range=1.5, max_range=40.0,
                                z_min=-0.8, z_max=2.5,
                                pred=None, gate=2.5, gate_ang=0.6,
                                eps=1.2, min_pts=3):
    """LiDAR 点云 (N,4)[x,y,z,intensity]（LiDAR 坐标系）→ 目标观测 (r, θ)。

    距离/高度过滤 → 体素降采样 → 欧式聚类，再按「半径 + 方位」双门限关联：
      - pred=(r_pred, θ_pred)：簇心到预测点的距离差 <= gate(m) **且**方位差 <= gate_ang(rad)；
        双门限内按 (距离差, 方位差) 字典序取最优（同距离不同方位时由方位决定）。
      - pred=(r_pred,)：仅半径门限（兼容旧调用）。
      - 未给 pred 时退回最近簇（bootstrap）。
    返回 None 表示本帧未检出。
    """
    if points is None or len(points) == 0:
        return None
    pts = np.asarray(points, dtype=float)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.hypot(x, y)
    keep = (r >= min_range) & (r <= max_range) & (z >= z_min) & (z <= z_max)
    if not np.any(keep):
        return None
    xy = np.stack([x[keep], y[keep]], axis=1)
    xy = voxel_downsample(xy, 0.4)
    clusters = cluster_points_2d(xy, eps=eps, min_pts=min_pts)
    if not clusters:
        return None
    if pred is not None:
        r_pred = float(pred[0])
        th_pred = float(pred[1]) if len(pred) > 1 else None
        cand = []
        for c in clusters:
            cr, cth = _polar(np.asarray(c[0]))
            dr = abs(cr - r_pred)
            dth = abs(wrap_angle(cth - th_pred)) if th_pred is not None else 0.0
            if dr > gate:
                continue
            if th_pred is not None and dth > gate_ang:
                continue
            cand.append((dr, dth, c))
        if not cand:
            return None
        best = min(cand, key=lambda t: (t[0], t[1]))[2]
        cxy = np.asarray(best[0])
    else:
        cxy = np.asarray(min(clusters, key=lambda c: np.hypot(*c[0]))[0])
    return _polar(cxy)


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
                                pred=None, gate=3.0, min_nearest=1, n_nearest=6):
    """CARLA Radar 原始检测点 (N,4)[depth, azimuth, altitude, velocity]
    → 目标观测 (r, θ)。

    CARLA sensor.other.radar 每点 4 个 float32：斜距 depth(m)、水平方位
    azimuth(rad，右为正)、俯仰 altitude(rad)、径向速度 velocity(m/s)。
    按 depth 过滤后做目标关联（评审点 4）：
      - pred=(r_pred, th_pred) 给出时，选离预测位置最近的、距离差 < gate 的点簇；
      - 未给 pred 时按深度做一维聚类，取「点数最多的回波簇」（bootstrap，
        避免目标回波与近处道路杂波一起取均值导致初值偏离）。
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
    if pred is None:
        # bootstrap：按深度做一维聚类，取「点数最多的回波簇」，避免杂波点稀释目标
        order = np.argsort(depth)
        groups, cur = [], [int(order[0])]
        for idx in order[1:]:
            idx = int(idx)
            if depth[idx] - depth[cur[-1]] <= 1.0:
                cur.append(idx)
            else:
                groups.append(cur)
                cur = [idx]
        groups.append(cur)
        best = max(groups, key=len)
        return float(depth[best].mean()), float(np.mean(azimuth[best]))

    pref = _xy(float(pred[0]), float(pred[1]))
    pts_xy = np.stack([depth * np.cos(azimuth), depth * np.sin(azimuth)], axis=1)
    d = np.hypot(pts_xy[:, 0] - pref[0], pts_xy[:, 1] - pref[1])
    dmin = float(d.min())
    if dmin > gate:
        return None
    sel = d <= max(dmin + 1.0, 1.5)          # 目标附近一簇点
    idx = np.where(sel)[0]
    idx = idx[np.argsort(d[idx])][:min(n_nearest, idx.size)]
    return float(depth[idx].mean()), float(np.mean(azimuth[idx]))
