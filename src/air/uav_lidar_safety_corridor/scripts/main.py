#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenHUTB/CarlaAir 无人机激光雷达安全走廊节点."""

from __future__ import annotations

import os
import sys
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller import (  # noqa: E402
    bearing_error_deg,
    body_frame_is_safe,
    compute_velocity,
)
from corridor import choose_altitude, distance_to_points  # noqa: E402
from depth_guard import (  # noqa: E402
    central_min_depth,
    depth_image_to_array,
    depth_speed_scale,
)
from metrics import FlightMetrics  # noqa: E402
from pointcloud import pointcloud2_to_xyz  # noqa: E402


def _as_goal(value):
    if isinstance(value, str):
        value = [
            float(item)
            for item in value.strip().strip("[]").replace(",", " ").split()
        ]
    return np.asarray(value, dtype=np.float64).reshape(3)


def _pose_position(odom):
    pose = odom.pose.pose.position
    return np.array([pose.x, pose.y, pose.z], dtype=np.float64)


def _pose_velocity(odom):
    twist = odom.twist.twist.linear
    return np.array([twist.x, twist.y, twist.z], dtype=np.float64)


def _pose_yaw(odom):
    q = odom.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SafetyCorridorNode:
    def __init__(self):
        self.goal_spec = _as_goal(rospy.get_param("nav/goal", [10.0, 0.0, 5.0]))
        self.relative_goal = bool(rospy.get_param("nav/relative_goal", False))
        self.goal = self.goal_spec.copy()
        self.goal_initialized = not self.relative_goal
        self.goal_checked = False
        self.goal_rejected = False
        self.max_goal_bearing = float(
            rospy.get_param("nav/max_goal_bearing_deg", 75.0)
        )
        self.rate_hz = float(rospy.get_param("control/rate_hz", 10.0))
        self.require_lidar = bool(
            rospy.get_param("control/require_lidar", True)
        )
        self.cloud_timeout = float(
            rospy.get_param("control/cloud_timeout", 0.8)
        )
        self.depth_enabled = bool(
            rospy.get_param("control/depth_enabled", True)
        )
        self.corridor_params = rospy.get_param("corridor", {})
        self.control_params = rospy.get_param("control", {})

        self.topic_odom = rospy.get_param("topics/odom", "/uav/odom")
        self.topic_lidar = rospy.get_param("topics/lidar", "/lidar/points")
        self.topic_depth = rospy.get_param("topics/depth", "/camera/depth")
        self.topic_cmd = rospy.get_param("topics/cmd_vel", "/uav/cmd_vel")
        self.topic_status = rospy.get_param(
            "topics/status", "/uav_lidar_safety_corridor/status"
        )
        self.topic_markers = rospy.get_param(
            "topics/markers", "/uav_lidar_safety_corridor/markers"
        )

        body_frame = rospy.get_param("control/body_frame", None)
        self.body_frame_safe = body_frame_is_safe(body_frame)
        if body_frame is None:
            rospy.logerr(
                "未读到 control/body_frame；安全策略将拒绝飞行。"
                "请使用 body_frame:=false 启动桥接"
            )
        elif bool(body_frame):
            rospy.logerr(
                "control/body_frame=true，但本节点发布世界 ENU 速度；"
                "安全策略将拒绝飞行，请用 body_frame:=false 重新启动桥接"
            )

        self.latest_odom = None
        self.latest_cloud = None
        self.latest_points = np.empty((0, 3), dtype=np.float64)
        self.latest_cloud_time = None
        self.latest_depth = None
        self.latest_depth_time = None
        self.metrics = FlightMetrics()
        self.finished = False
        self.metrics_written = False
        self.done_logged = False

        self.cmd_pub = rospy.Publisher(self.topic_cmd, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.topic_status, String, queue_size=1)
        self.marker_pub = rospy.Publisher(
            self.topic_markers, MarkerArray, queue_size=1
        )
        rospy.Subscriber(
            self.topic_odom, Odometry, self._odom_callback, queue_size=1
        )
        rospy.Subscriber(
            self.topic_lidar, PointCloud2, self._cloud_callback, queue_size=1
        )
        rospy.Subscriber(
            self.topic_depth, Image, self._depth_callback, queue_size=1
        )
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "uav_lidar_safety_corridor: goal ENU=%s, odom=%s, lidar=%s",
            self.goal.tolist(), self.topic_odom, self.topic_lidar,
        )

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _cloud_callback(self, msg):
        try:
            self.latest_points = pointcloud2_to_xyz(msg)
            self.latest_cloud = msg
            self.latest_cloud_time = msg.header.stamp.to_sec()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(3.0, "点云解析失败: %s", exc)

    def _depth_callback(self, msg):
        if not self.depth_enabled:
            return
        try:
            self.latest_depth = depth_image_to_array(msg)
            self.latest_depth_time = msg.header.stamp.to_sec()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(3.0, "深度图解析失败: %s", exc)

    def _cloud_ready(self):
        if not self.require_lidar:
            return True
        if self.latest_cloud is None or self.latest_cloud_time is None:
            return False
        return (rospy.Time.now().to_sec() - self.latest_cloud_time) <= self.cloud_timeout

    def _front_depth(self):
        if not self.depth_enabled:
            return float("inf")
        if self.latest_depth is None or self.latest_depth_time is None:
            return float("inf")
        age = rospy.Time.now().to_sec() - self.latest_depth_time
        if age > float(self.control_params.get("depth_timeout", 0.8)):
            return float("inf")
        return central_min_depth(
            self.latest_depth,
            self.control_params.get("depth_roi_x_min", 0.2),
            self.control_params.get("depth_roi_x_max", 0.8),
            self.control_params.get("depth_roi_y_min", 0.25),
            self.control_params.get("depth_roi_y_max", 0.75),
        )

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo_throttle(1.0, "%s", text)

    def _publish_markers(self, position, choice):
        direction = self.goal[:2] - position[:2]
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            direction = np.array([1.0, 0.0], dtype=np.float64)
        else:
            direction /= norm
        preview = Point(
            x=float(position[0] + direction[0] * 4.0),
            y=float(position[1] + direction[1] * 4.0),
            z=float(choice.target_z),
        )
        start = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )

        sphere = Marker()
        sphere.header.frame_id = "world"
        sphere.header.stamp = rospy.Time.now()
        sphere.ns = "safe_corridor"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = preview
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
        sphere.color.r = 0.1
        sphere.color.g = 0.9
        sphere.color.b = 0.2
        sphere.color.a = 0.8

        line = Marker()
        line.header = sphere.header
        line.ns = "safe_corridor"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r = 0.1
        line.color.g = 0.8
        line.color.b = 0.2
        line.color.a = 0.8
        line.points = [start, preview]

        text = Marker()
        text.header = sphere.header
        text.ns = "safe_corridor"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = preview
        text.pose.position.z += 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        text.text = "safe z=%.2f m" % choice.target_z

        self.marker_pub.publish(MarkerArray(markers=[sphere, line, text]))

    def _step_once(self):
        if not self.body_frame_safe:
            self._publish_stop()
            self._publish_status("REJECTED body_frame_not_false")
            return
        if self.latest_odom is None:
            self._publish_stop()
            self._publish_status("WAIT_ODOM")
            return
        if not self._cloud_ready():
            self._publish_stop()
            self._publish_status("WAIT_LIDAR")
            return

        position = _pose_position(self.latest_odom)
        velocity = _pose_velocity(self.latest_odom)
        if not self.goal_initialized:
            self.goal = position + self.goal_spec
            self.goal_initialized = True
            rospy.loginfo("相对目标已锁定: goal ENU=%s", self.goal.tolist())
        if not self.goal_checked:
            bearing = bearing_error_deg(position, self.goal, _pose_yaw(self.latest_odom))
            self.goal_checked = True
            if bearing > self.max_goal_bearing:
                self.goal_rejected = True
                rospy.logerr(
                    "目标方向偏离机头 %.1f°，超过前向安全走廊限制 %.1f°",
                    bearing,
                    self.max_goal_bearing,
                )
        if self.goal_rejected:
            self._publish_stop()
            self._publish_status(
                "REJECTED max_goal_bearing=%.1f" % self.max_goal_bearing
            )
            return
        points = self.latest_points
        choice = choose_altitude(points, position, self.goal, self.corridor_params)
        front_depth = self._front_depth()
        depth_scale = depth_speed_scale(
            front_depth,
            self.control_params.get("depth_stop_distance", 1.2),
            self.control_params.get("depth_slow_distance", 3.5),
        )
        command = compute_velocity(
            position,
            velocity,
            self.goal,
            choice.target_z,
            points,
            self.control_params,
            1.0 / max(self.rate_hz, 1e-3),
            depth_scale,
        )

        msg = Twist()
        msg.linear.x = command.vx
        msg.linear.y = command.vy
        msg.linear.z = command.vz
        self.cmd_pub.publish(msg)
        self._publish_markers(position, choice)
        self.metrics.update(position, self.goal, points, command)

        distance = float(np.linalg.norm(self.goal - position))
        reached = (
            np.linalg.norm(self.goal[:2] - position[:2])
            <= float(self.control_params.get("goal_tolerance_xy", 1.0))
            and abs(self.goal[2] - position[2])
            <= float(self.control_params.get("goal_tolerance_z", 0.7))
        )
        if reached:
            self.finished = True
            if not self.done_logged:
                self._publish_status("DONE goal_dist=%.2f" % distance)
                self.done_logged = True
            return

        self._publish_status(
            "RUN target_z=%.2f risk=%.3f corridor_points=%d "
            "nearest=%.2f depth=%.2f speed_scale=%.2f goal_dist=%.2f"
            % (
                choice.target_z,
                choice.risk,
                choice.corridor_points,
                command.nearest_distance,
                front_depth,
                command.speed_scale,
                distance,
            )
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self._step_once()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(2.0, "控制循环异常: %s", exc)
                self._publish_stop()
                self._publish_status("ERROR %s" % exc)
            rate.sleep()

    def _shutdown(self):
        self._publish_stop()
        if self.metrics_written:
            return
        self.metrics_written = True
        path = rospy.get_param("metrics/output_file", "")
        try:
            self.metrics.write(path)
            if path:
                rospy.loginfo("飞行指标已写入: %s", os.path.expanduser(path))
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("飞行指标写入失败: %s", exc)


def main():
    rospy.init_node("uav_lidar_safety_corridor", anonymous=False)
    SafetyCorridorNode().run()


if __name__ == "__main__":
    main()
