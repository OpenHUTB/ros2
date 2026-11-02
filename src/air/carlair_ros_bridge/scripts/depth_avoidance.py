#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度避障节点：/camera/depth/points -> /uav/cmd_vel.

策略（相机光学系：x 右、y 下、z 前）：

1. 取前方走廊内的点：|x|<=lateral_limit、|y|<=vertical_limit、z<=max_range；
2. 最近距离 d = min(z)；
3. d <= stop_distance：停车悬停；
   stop_distance < d < slow_distance：按线性比例减速；
   d >= slow_distance：按 forward_speed 前进。

这是反应式安全层，不替代 uav_ppo_nav / octree 的全局规划；
默认关闭，demo 时通过 enable_depth_avoidance:=true 打开。

参数（默认值见 config/bridge.yaml）：
    depth_avoidance/enabled, forward_speed, slow_distance, stop_distance
    depth_avoidance/lateral_limit, vertical_limit, max_range, rate
    topic/depth_points, topic/cmd_vel
"""
from __future__ import annotations

import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def corridor_points(points, lateral_limit, vertical_limit, max_range, min_range=0.0):
    """筛选前方走廊内的点（相机光学系），返回 (N,3) float32."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    mask = (
        (np.abs(pts[:, 0]) <= lateral_limit)
        & (np.abs(pts[:, 1]) <= vertical_limit)
        & (pts[:, 2] >= min_range)
        & (pts[:, 2] <= max_range)
    )
    return pts[mask]


def nearest_distance(points, lateral_limit, vertical_limit, max_range, min_range=0.0):
    """走廊内最近点的前向距离；无点时返回 inf."""
    pts = corridor_points(points, lateral_limit, vertical_limit, max_range, min_range)
    if pts.shape[0] == 0:
        return float("inf")
    return float(np.min(pts[:, 2]))


def compute_speed(distance, stop_distance, slow_distance, max_speed):
    """根据最近距离给出前向速度（m/s）."""
    if math.isinf(distance):
        return float(max_speed)
    if not math.isfinite(distance) or distance <= stop_distance:
        return 0.0
    if distance >= slow_distance:
        return float(max_speed)
    ratio = (distance - stop_distance) / max(1e-6, slow_distance - stop_distance)
    return float(max_speed * max(0.0, min(1.0, ratio)))


class DepthAvoidance(object):
    def __init__(self):
        self.enabled = bool(rospy.get_param("depth_avoidance/enabled", True))
        self.forward_speed = float(rospy.get_param("depth_avoidance/forward_speed", 1.5))
        self.slow_distance = float(rospy.get_param("depth_avoidance/slow_distance", 4.0))
        self.stop_distance = float(rospy.get_param("depth_avoidance/stop_distance", 2.0))
        self.lateral_limit = float(rospy.get_param("depth_avoidance/lateral_limit", 0.8))
        self.vertical_limit = float(rospy.get_param("depth_avoidance/vertical_limit", 0.8))
        self.max_range = float(rospy.get_param("depth_avoidance/max_range", 12.0))
        self.min_range = float(rospy.get_param("depth_avoidance/min_range", 0.5))
        self.start_delay = float(rospy.get_param("depth_avoidance/start_delay", 6.0))
        self.rate = rospy.Rate(float(rospy.get_param("depth_avoidance/rate", 10.0)))
        self.start_time = time.time()

        self.topic_points = rospy.get_param("topic/depth_points", "/camera/depth/points")
        self.topic_cmd = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        self.pub = rospy.Publisher(self.topic_cmd, Twist, queue_size=1)
        self.sub = rospy.Subscriber(
            self.topic_points, PointCloud2, self.cloud_cb, queue_size=1
        )
        self.points = None
        self.last_action = ""
        rospy.loginfo(
            "depth_avoidance: stop=%.1fm slow=%.1fm speed=%.1fm/s",
            self.stop_distance, self.slow_distance, self.forward_speed,
        )

    def cloud_cb(self, msg):
        pts = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))
        self.points = np.asarray(pts, dtype=np.float32) if pts else np.empty((0, 3), np.float32)

    def step_once(self):
        if not self.enabled or self.points is None:
            return
        # 等自动起飞完成后再接管，避免把爬升中的速度打成 0
        if time.time() - self.start_time < self.start_delay:
            return
        d = nearest_distance(
            self.points, self.lateral_limit, self.vertical_limit,
            self.max_range, self.min_range)
        vx = compute_speed(
            d, self.stop_distance, self.slow_distance, self.forward_speed)
        msg = Twist()
        msg.linear.x = vx
        self.pub.publish(msg)

        action = "STOP" if vx == 0.0 else ("SLOW" if vx < self.forward_speed else "GO")
        if action != self.last_action:
            rospy.loginfo("depth_avoidance: d=%.2fm vx=%.2f -> %s", d, vx, action)
            self.last_action = action

    def spin(self):
        while not rospy.is_shutdown():
            try:
                self.step_once()
            except Exception as exc:  # noqa: BLE001 - 断连时保持节点存活
                rospy.logwarn_throttle(5.0, "depth_avoidance: %s", exc)
            self.rate.sleep()


def main():
    rospy.init_node("uav_depth_avoidance", anonymous=False)
    DepthAvoidance().spin()


if __name__ == "__main__":
    main()
