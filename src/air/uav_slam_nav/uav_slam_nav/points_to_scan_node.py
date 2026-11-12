#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""世界系 LiDAR PointCloud2 -> base_link 系 LaserScan（ROS2/rclpy 版）。

做法：
  1. 订阅 /lidar/points（Noetic bridge 发出来，经 ros1_bridge 跨到 Humble）。
  2. 用 tf2_ros.Buffer 查 cloud header.frame_id -> base_link 的变换，
     不再自己拿 /uav/odom 位姿做逆变换（这样 SLAM 校正后的 map->odom 也能被吃到）。
  3. 纯投影逻辑在 nav_logic.project_points_to_scan。
  4. 发布 /scan 给 slam_toolbox / nav2。
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener

from . import nav_logic


def parse_xyz(msg: PointCloud2) -> np.ndarray:
    """从 PointCloud2 里抠出 (N,3) 的 xyz。只找 FLOAT32/FLOAT64 的 x/y/z 字段。"""
    fields = {f.name: f for f in msg.fields}
    needed = ("x", "y", "z")
    if not all(k in fields for k in needed):
        return np.zeros((0, 3), dtype=np.float64)
    offsets = {k: fields[k].offset for k in needed}
    dtype = np.float32 if fields["x"].datatype == 7 else np.float64
    n = msg.width * msg.height
    if n == 0 or msg.point_step == 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float64)
    for i, k in enumerate(needed):
        out[:, i] = raw[:, offsets[k]:offsets[k] + 4].copy().view(dtype).reshape(-1)
    return out


class PointsToScan(Node):
    def __init__(self):
        super().__init__("points_to_scan")
        self.declare_parameter("height_lo", -0.3)
        self.declare_parameter("height_hi", 3.0)
        self.declare_parameter("range_min", 0.3)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("angle_min", -np.pi)
        self.declare_parameter("angle_max", np.pi)
        self.declare_parameter("angle_increment", np.deg2rad(1.0))
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("timeout", 0.1)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.sub = self.create_subscription(PointCloud2, "/lidar/points", self.cloud_cb, 10)
        self._last_pub_t = self.get_clock().now()

    def cloud_cb(self, msg: PointCloud2):
        now = self.get_clock().now()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("base_frame").value,
                msg.header.frame_id or "world",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.get_parameter("timeout").value),
            )
        except Exception as e:
            self.get_logger().warn_throttle(3.0, f"tf lookup failed: {e}")
            return

        pts = parse_xyz(msg)
        T = nav_logic.transform_stamped_to_matrix(tf.transform)
        ranges = nav_logic.project_points_to_scan(
            pts, T,
            angle_min=self.get_parameter("angle_min").value,
            angle_max=self.get_parameter("angle_max").value,
            angle_inc=self.get_parameter("angle_increment").value,
            range_min=self.get_parameter("range_min").value,
            range_max=self.get_parameter("range_max").value,
            height_lo=self.get_parameter("height_lo").value,
            height_hi=self.get_parameter("height_hi").value,
        )
        if ranges.size == 0:
            return

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.get_parameter("base_frame").value
        scan.angle_min = float(self.get_parameter("angle_min").value)
        scan.angle_max = float(self.get_parameter("angle_max").value)
        scan.angle_increment = float(self.get_parameter("angle_increment").value)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = float(self.get_parameter("range_min").value)
        scan.range_max = float(self.get_parameter("range_max").value)
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)
        self._last_pub_t = now


def main():
    rclpy.init()
    node = PointsToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
