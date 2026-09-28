#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 导航部署节点：订阅 /uav/odom + /lidar/points，发布 /uav/cmd_vel.

用训练好的 MLP 策略（policy_weights.npz）做纯 numpy 推理，部署侧不需要 torch/sb3。
目标点由 rosparam ``nav/goal`` 指定（ENU 世界系，默认 [30, 10, -8]）。

依赖：
    需要 carlair_ros_bridge 已启动并发布 /uav/odom、/lidar/points，
    同时订阅 /uav/cmd_vel 执行速度指令。
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy import (MlpPolicy, action_to_velocity, build_observation,  # noqa: E402
                    lidar_points_to_histogram, world_to_body)


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N, 3) float64 世界系点（假设字段为 xyz float32）."""
    pts = np.frombuffer(msg.data, dtype=np.float32)
    return pts.reshape(-1, msg.point_step // 4)[:, :3].astype(np.float64)


class PpoNavNode(object):
    def __init__(self):
        weights_path = rospy.get_param("nav/weights", "")
        if not weights_path:
            import rospkg  # noqa: WPS433
            weights_path = os.path.join(rospkg.RosPack().get_path("uav_ppo_nav"),
                                        "models", "policy_weights.npz")
        self.policy = MlpPolicy.from_npz(weights_path)
        rospy.loginfo("ppo_nav: 已加载策略 %s", weights_path)

        self.goal = np.asarray(rospy.get_param("nav/goal", [30.0, 10.0, -8.0]),
                               dtype=np.float64)
        self.publish_hz = float(rospy.get_param("rate/publish_hz", 10.0))
        self.cmd_topic = rospy.get_param("topic/cmd_vel", "/uav/cmd_vel")
        rospy.loginfo("ppo_nav: 目标点 ENU %s", self.goal)

        self.latest_odom = None
        self.latest_cloud = None
        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber("/uav/odom", Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber("/lidar/points", PointCloud2, self._cb_cloud, queue_size=1)
        self.rate = rospy.Rate(self.publish_hz)

    def _cb_odom(self, msg):
        self.latest_odom = msg

    def _cb_cloud(self, msg):
        self.latest_cloud = msg

    # ------------------------------------------------------------------ 观测
    def build_obs(self) -> np.ndarray:
        odom = self.latest_odom
        pos = np.array([odom.pose.pose.position.x,
                        odom.pose.pose.position.y,
                        odom.pose.pose.position.z], dtype=np.float64)
        yaw = yaw_from_odom(odom)
        vel_world = np.array([odom.twist.twist.linear.x,
                              odom.twist.twist.linear.y,
                              odom.twist.twist.linear.z], dtype=np.float64)

        if self.latest_cloud is not None:
            pts = cloud_to_xyz(self.latest_cloud)
            rel = pts - pos
            c, s = math.cos(yaw), math.sin(yaw)
            body = np.stack([rel[:, 0] * c + rel[:, 1] * s,
                             -rel[:, 0] * s + rel[:, 1] * c,
                             rel[:, 2]], axis=1)
            hist = lidar_points_to_histogram(body)
        else:
            hist = np.ones(16, dtype=np.float32)   # 尚无点云 -> 视为无遮挡

        goal_body = world_to_body(self.goal - pos, yaw)
        vel_body = world_to_body(vel_world, yaw)
        return build_observation(hist, goal_body, vel_body)

    # ------------------------------------------------------------------ 主循环
    def step_once(self):
        """读一次观测 -> 策略推理 -> 发布速度指令（独立成函数便于测试）."""
        obs = self.build_obs()
        action = self.policy.forward(obs)
        vx, vy, vz, wz = action_to_velocity(action)
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = vx, vy, vz
        t.angular.z = wz
        self.pub.publish(t)
        return obs, action

    def spin(self):
        while not rospy.is_shutdown():
            if self.latest_odom is None:
                rospy.loginfo_throttle(3.0, "ppo_nav: 等待 /uav/odom ...")
                self.rate.sleep()
                continue
            try:
                self.step_once()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(3.0, "ppo_nav: %s", exc)
            self.rate.sleep()


def main():
    rospy.init_node("uav_ppo_nav", anonymous=False)
    PpoNavNode().spin()


if __name__ == "__main__":
    main()
