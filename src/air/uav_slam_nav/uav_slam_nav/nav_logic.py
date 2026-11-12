"""uav_slam_nav 的纯计算模块。

这里只放数学和判断逻辑，不 import rclpy / rospy，目的是让 test/ 下的
单元测试在没有 ROS 的机器上也能直接跑（CI 就是这么跑的）。节点只负责
收发话题，判断逻辑全部收在这里。
"""
from __future__ import annotations

import numpy as np

# nav2 / action_msgs 的目标终态里算“失败”的集合
GOAL_SUCCEEDED = 3
GOAL_ABORTED = 4
GOAL_REJECTED = 5
GOAL_RECALLED = 8
GOAL_LOST = 9

FAILED_STATUSES = (GOAL_ABORTED, GOAL_REJECTED, GOAL_RECALLED, GOAL_LOST)

DEFAULT_FALLBACK_CFG = {
    "stop_distance": 1.0,     # 前向扇区最近障碍小于这个距离就停车转向
    "turn_rate": 0.6,         # 避障转向角速度 rad/s
    "max_vx": 0.5,            # 兜底前进速度 m/s
    "max_yaw": 0.6,           # 兜底角速度上限 rad/s
    "goal_gain": 1.0,         # 方位误差 -> 角速度 的比例
    "align_slow_rad": 1.2,    # 方位误差大于这个值就只转不走（约 69 度）
}


def clamp(v, lo, hi):
    if hi < lo:
        raise ValueError("clamp: hi 必须 >= lo")
    return max(lo, min(hi, float(v)))


def status_is_failed(code):
    return int(code) in FAILED_STATUSES


def normalize_angle(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


def quat_to_matrix(quat):
    x, y, z, w = [float(v) for v in quat]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def yaw_from_quat(quat):
    x, y, z, w = [float(v) for v in quat]
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def project_points_to_scan(pts_cloud, transform, angle_min, angle_max, angle_inc,
                           range_min, range_max, height_lo, height_hi):
    """把一帧世界系点云投成 base_link 系 LaserScan ranges。

    pts_cloud: (N,3) ndarray，点在点云自身坐标系下。
    transform: (4,4) ndarray，base_link <- cloud 的齐次变换（TF 查出来的）。
    按 base_link 下的 z 切片，按方位角分 bin，每个 bin 取最近距离。
    全部落在量程外时返回空数组，调用方跳过这一帧。
    """
    pts = np.asarray(pts_cloud, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] != 3:
        return np.array([], dtype=np.float64)
    if angle_inc <= 0:
        raise ValueError("angle_inc 必须 > 0")

    T = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    p_b = (R @ pts.T).T + t                       # (N,3) base 系

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
    order = np.argsort(r)
    idx_sorted = idx[order]
    r_sorted = r[order]
    _, first = np.unique(idx_sorted, return_index=True)
    ranges[idx_sorted[first]] = r_sorted[first]
    return ranges


def sector_min_range(ranges, angle_min, angle_increment, lo, hi,
                     range_min=0.0, range_max=float("inf")):
    """某个角度区间内的最近有效距离。

    LaserScan 里填 range_max 的 bin 表示"没有回波"，要排除掉；
    全是无效回波时返回 None。
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return None
    ang = angle_min + np.arange(r.size, dtype=np.float64) * angle_increment
    mask = ((ang >= lo) & (ang <= hi) & np.isfinite(r)
            & (r >= range_min) & (r < range_max - 1e-6))
    if not np.any(mask):
        return None
    return float(r[mask].min())


def front_min_range(ranges, angle_min, angle_increment, half_width_rad,
                    range_min=0.0, range_max=float("inf")):
    return sector_min_range(ranges, angle_min, angle_increment,
                            -abs(half_width_rad), abs(half_width_rad),
                            range_min, range_max)


def side_min_range(ranges, angle_min, angle_increment, half_width_rad,
                   range_min=0.0, range_max=float("inf")):
    """左右两侧的最近距离，返回 (left_min, right_min)。"""
    left = sector_min_range(ranges, angle_min, angle_increment,
                            half_width_rad, half_width_rad * 2.0,
                            range_min, range_max)
    right = sector_min_range(ranges, angle_min, angle_increment,
                             -half_width_rad * 2.0, -half_width_rad,
                             range_min, range_max)
    return left, right


def choose_turn_sign(left_min, right_min):
    left_clear = float("inf") if left_min is None else float(left_min)
    right_clear = float("inf") if right_min is None else float(right_min)
    if right_clear > left_clear:
        return -1
    return 1


def decide_command(front_min, left_min, right_min, goal_bearing, cfg=None):
    """兜底决策，返回 (vx, wz)。"""
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
    return (clamp(vx, -max_vx, max_vx),
            clamp(vy, -max_vy, max_vy),
            0.0,
            clamp(wz, -max_yaw, max_yaw))


def select_source(now, nav_stamp, fb_stamp, stale_s):
    """兜底优先；两边都超时返回 None，让 bridge 自己悬停。"""
    if fb_stamp is not None and 0 <= now - fb_stamp <= stale_s:
        return "fallback"
    if nav_stamp is not None and 0 <= now - nav_stamp <= stale_s:
        return "nav2"
    return None


def transform_stamped_to_matrix(tf_msg):
    """把 geometry_msgs/TransformStamped 转成 4x4 矩阵（base <- cloud）。

    纯 numpy 实现，不依赖 tf2_py，方便单测。
    """
    q = tf_msg.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    t = np.array([tf_msg.translation.x, tf_msg.translation.y, tf_msg.translation.z])
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T
