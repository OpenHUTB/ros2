#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""世界系 LiDAR 点云 -> base_link 系 LaserScan（直接用 /uav/odom 位姿做逆变换）。

桥接包发的 `/lidar/points` 是世界系（ENU）PointCloud2，`/uav/odom` 是同坐标系下
的位姿。本节点用 odom 的 position+orientation 直接算逆变换，把点云拉到 base_link，
再按机体水平面切片投影成 360° LaserScan，供 RTAB-Map / 2D SLAM 使用。

参数（launch 覆盖）：
    ~range_min         默认 0.3 m
    ~range_max         默认 30.0 m
    ~angle_min         默认 -pi
    ~angle_max         默认 +pi
    ~angle_increment   默认 1°（弧度）
    ~height_lo        默认 -0.3 m（base_link 系下点云允许的最低 z，滤地面）
    ~height_hi        默认  3.0 m（base_link 系下点云允许的最高 z，滤高空楼）
"""
from __future__ import annotations

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2


def project(pts_world: np.ndarray, pos: np.ndarray, quat: np.ndarray,
            angle_min: float, angle_max: float, angle_inc: float,
            range_min: float, range_max: float,
            height_lo: float, height_hi: float) -> np.ndarray:
    """世界系点云 (N,3) -> LaserScan ranges。纯函数，便于无 ROS 测试。"""
    n = pts_world.shape[0]
    if n == 0:
        return np.array([], dtype=np.float64)

    x, y, z, w = quat
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)

    p_rel = pts_world - pos                       # (N,3)
    p_b = (R.T @ p_rel.T).T                       # (N,3) base 系

    mask = (p_b[:, 2] >= height_lo) & (p_b[:, 2] <= height_hi)
    p_b = p_b[mask]
    if p_b.shape[0] == 0:
        return np.array([], dtype=np.float64)

    r = np.hypot(p_b[:, 0], p_b[:, 1])
    ang = np.arctan2(p_b[:, 1], p_b[:, 0])

    n_bins = int(round((angle_max - angle_min) / angle_inc)) + 1
    ranges = np.full(n_bins, range_max, dtype=np.float64)

    order = np.argsort(r)
    idx = np.floor((ang - angle_min) / angle_inc).astype(int)
    idx = np.clip(idx, 0, n_bins - 1)
    seen = np.zeros(n_bins, dtype=bool)
    for i in order:
        bi = int(idx[i])
        if seen[bi]:
            continue
        if range_min <= r[i] <= range_max:
            seen[bi] = True
            ranges[bi] = float(r[i])
    return ranges


class PointsToScan(object):
    def __init__(self):
        self.range_min = float(rospy.get_param("~range_min", 0.3))
        self.range_max = float(rospy.get_param("~range_max", 30.0))
        self.angle_min = float(rospy.get_param("~angle_min", -np.pi))
        self.angle_max = float(rospy.get_param("~angle_max", np.pi))
        self.angle_inc = float(rospy.get_param("~angle_increment", np.deg2rad(1.0)))
        self.height_lo = float(rospy.get_param("~height_lo", -0.3))
        self.height_hi = float(rospy.get_param("~height_hi", 3.0))

        self.pos = None
        self.quat = None

        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=1)
        rospy.Subscriber("/uav/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/lidar/points", PointCloud2, self.cloud_cb, queue_size=1)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

    def cloud_cb(self, cloud: PointCloud2):
        if self.pos is None:
            return
        pts = np.array(list(pc2.read_points(cloud, field_names=("x", "y", "z"),
                                            skip_nans=True)), dtype=np.float64)
        ranges = project(pts, self.pos, self.quat,
                         self.angle_min, self.angle_max, self.angle_inc,
                         self.range_min, self.range_max,
                         self.height_lo, self.height_hi)
        if ranges.size == 0:
            return

        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rospy.init_node("points_to_scan")
    PointsToScan()
    rospy.spin()


if __name__ == "__main__":
    main()
