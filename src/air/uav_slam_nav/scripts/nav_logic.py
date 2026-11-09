#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_slam_nav 的纯计算模块。

这里只放数学和判断逻辑，不 import rospy，目的是让 tests/ 下的单元测试
在没有 ROS 的机器上也能直接跑（CI 就是这么跑的）。三个节点只负责收发话题，
判断逻辑全部收在这里。
"""
from __future__ import annotations

import numpy as np

# actionlib_msgs/GoalStatus 的状态码
STATUS_PENDING = 0
STATUS_ACTIVE = 1
STATUS_PREEMPTED = 2
STATUS_SUCCEEDED = 3
STATUS_ABORTED = 4
STATUS_REJECTED = 5
STATUS_PREEMPTING = 6
STATUS_RECALLING = 7
STATUS_RECALLED = 8
STATUS_LOST = 9

# move_base 当前目标算“失败”，兜底才接管
FAILED_STATUSES = (STATUS_ABORTED, STATUS_REJECTED, STATUS_RECALLED, STATUS_LOST)

DEFAULT_FALLBACK_CFG = {
    "stop_distance": 1.0,     # 前向扇区最近障碍小于这个距离就停车转向
    "turn_rate": 0.6,         # 避障转向角速度 rad/s
    "max_vx": 0.5,            # 兜底前进速度 m/s
    "max_yaw": 0.6,           # 兜底角速度上限 rad/s
    "goal_gain": 1.0,         # 方位误差 -> 角速度 的比例
    "align_slow_rad": 1.2,    # 方位误差大于这个值就只转不走（约 69°）
}


def clamp(v, lo, hi):
    """把 v 夹到 [lo, hi]。"""
    if hi < lo:
        raise ValueError("clamp: hi 必须 >= lo")
    return max(lo, min(hi, float(v)))


def status_is_failed(code):
    """move_base 某个目标的 status 是不是失败态（规划失败/放弃/丢失）。"""
    return int(code) in FAILED_STATUSES


def normalize_angle(a):
    """把角度折到 [-pi, pi]。"""
    return float(np.arctan2(np.sin(a), np.cos(a)))


def quat_to_matrix(quat):
    """四元数 (x, y, z, w) -> 旋转矩阵，避免额外依赖 tf.transformations。"""
    x, y, z, w = [float(v) for v in quat]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_from_quat(quat):
    """只取偏航角。"""
    x, y, z, w = [float(v) for v in quat]
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def project(pts_world, pos, quat, angle_min, angle_max, angle_inc,
            range_min, range_max, height_lo, height_hi):
    """世界系点云 (N,3) -> base_link 系 LaserScan ranges。

    步骤：世界系点减去 odom 位置，再用姿态旋转的转置转到 base_link；
    按 base_link 下的 z 切片滤掉地面和高空；按方位角分 bin，每个 bin 取最近的距离。
    全部落在 [range_min, range_max] 之外时返回空数组，调用方跳过这一帧。
    """
    pts = np.asarray(pts_world, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] != 3:
        return np.array([], dtype=np.float64)
    if angle_inc <= 0:
        raise ValueError("angle_inc 必须 > 0")

    R = quat_to_matrix(quat)
    pos = np.asarray(pos, dtype=np.float64).reshape(3)

    p_b = (R.T @ (pts - pos).T).T                       # (N,3) base 系
    mask_h = (p_b[:, 2] >= height_lo) & (p_b[:, 2] <= height_hi)
    p_b = p_b[mask_h]
    if p_b.shape[0] == 0:
        return np.array([], dtype=np.float64)

    r = np.hypot(p_b[:, 0], p_b[:, 1])
    ang = np.arctan2(p_b[:, 1], p_b[:, 0])

    n_bins = int(round((angle_max - angle_min) / angle_inc)) + 1
    in_range = (r >= range_min) & (r <= range_max)
    if not np.any(in_range):
        return np.array([], dtype=np.float64)

    r = r[in_range]
    ang = ang[in_range]
    idx = np.clip(np.floor((ang - angle_min) / angle_inc).astype(int), 0, n_bins - 1)

    ranges = np.full(n_bins, float(range_max), dtype=np.float64)
    # 按距离升序排，每个 bin 只保留第一次出现的点（即最近的那个）
    order = np.argsort(r)
    idx_sorted = idx[order]
    r_sorted = r[order]
    _, first = np.unique(idx_sorted, return_index=True)
    ranges[idx_sorted[first]] = r_sorted[first]
    return ranges


def sector_min_range(ranges, angle_min, angle_increment, lo, hi, range_min=0.0):
    """某个角度区间内的最近有效距离，没有有效点时返回 None。"""
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return None
    ang = angle_min + np.arange(r.size, dtype=np.float64) * angle_increment
    mask = (ang >= lo) & (ang <= hi) & np.isfinite(r) & (r >= range_min)
    if not np.any(mask):
        return None
    return float(r[mask].min())


def front_min_range(ranges, angle_min, angle_increment, half_width_rad, range_min=0.0):
    """前向扇区（|angle| <= half_width_rad）的最近障碍距离。"""
    return sector_min_range(ranges, angle_min, angle_increment,
                            -abs(half_width_rad), abs(half_width_rad), range_min)


def choose_turn_sign(left_min, right_min):
    """前面撞墙时往哪边转：返回 +1 左转，-1 右转。

    两边都没数据时默认左转；一边没数据就当作那一边是空的。
    """
    left_clear = float("inf") if left_min is None else float(left_min)
    right_clear = float("inf") if right_min is None else float(right_min)
    if right_clear > left_clear:
        return -1
    return 1


def decide_command(front_min, left_min, right_min, goal_bearing, cfg=None):
    """兜底节点的决策，返回 (vx, wz)。

    - 前方太近：停车并向空间大的一侧转
    - 没有目标方位：原地不动（兜底也不瞎飞）
    - 有目标方位：朝目标修角，偏差大时先转不走
    """
    c = dict(DEFAULT_FALLBACK_CFG)
    if cfg:
        c.update(cfg)

    if front_min is not None and front_min < c["stop_distance"]:
        return 0.0, choose_turn_sign(left_min, right_min) * c["turn_rate"]

    if goal_bearing is None:
        return 0.0, 0.0

    err = normalize_angle(goal_bearing)
    wz = clamp(c["goal_gain"] * err, -c["max_yaw"], c["max_yaw"])
    slow = max(float(c["align_slow_rad"]), 1e-6)
    vx = c["max_vx"] * max(0.0, 1.0 - abs(err) / slow)
    return vx, wz


def limit_twist(vx, vy, wz, max_vx, max_vy, max_yaw):
    """速度限幅，并把 z 速度强制清零（定高交给 bridge）。"""
    return clamp(vx, -max_vx, max_vx), clamp(vy, -max_vy, max_vy), 0.0, clamp(wz, -max_yaw, max_yaw)


def select_source(now, mb_stamp, fb_stamp, stale_s):
    """决定这一拍听谁的：'fallback' / 'move_base' / None（都不发）。

    兜底优先，但只在它还在持续输出时有效；两边都超时就返回 None，
    让 bridge 自己的超时悬停接管。
    """
    if fb_stamp is not None and 0 <= now - fb_stamp <= stale_s:
        return "fallback"
    if mb_stamp is not None and 0 <= now - mb_stamp <= stale_s:
        return "move_base"
    return None
