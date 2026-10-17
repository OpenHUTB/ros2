#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 封装：把 Stanley 横向控制器接到真实话题上。

订阅
----
* ``/reference_path`` (nav_msgs/Path)  —— 参考路径，也可由参数 ``reference_scenario`` 内置生成
* ``/odom`` (nav_msgs/Odometry)        —— 车辆位姿与速度

发布
----
* ``/cmd_vel`` (geometry_msgs/Twist)          —— 线速度与横摆角速度，可直接驱动差速小车
* ``/stanley/steering`` (std_msgs/Float64)    —— 期望前轮转角，便于阿克曼底盘或调试观察
* ``/stanley/tracking_error`` (std_msgs/Float64MultiArray) —— [横向偏差, 航向偏差(rad), 转向角, 期望速度]

参数见 ``config/params.yaml``。ROS 2 不可用时本文件不会被导入，
因此 ``main.py`` 的离线模式与单元测试都不需要安装 ROS。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

from .controllers import build_controller
from .geometry import ReferencePath, wrap_to_pi
from .reference import build_scenario
from .simulator import SpeedProfile


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数转偏航角（只取绕 z 轴的部分）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class StanleyTrackingNode(Node):
    """Stanley 跟踪节点：位姿 → 横向偏差 → 横摆角速度指令。"""

    def __init__(self) -> None:
        super().__init__("stanley_tracking_node")

        self.declare_parameter("controller", "stanley")
        self.declare_parameter("wheelbase", 2.7)
        self.declare_parameter("k", 1.6)
        self.declare_parameter("k_soft", 1.2)
        self.declare_parameter("feedforward", True)
        self.declare_parameter("max_steering_deg", 35.0)
        self.declare_parameter("steering_rate_deg", 120.0)
        self.declare_parameter("target_speed", 5.0)
        self.declare_parameter("min_speed", 1.5)
        self.declare_parameter("a_lat_max", 2.5)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("reference_scenario", "circle")
        self.declare_parameter("wait_for_path", True)

        controller_name = str(self.get_parameter("controller").value)
        self.steering_rate_limit = math.radians(
            float(self.get_parameter("steering_rate_deg").value))
        self.controller = build_controller(
            controller_name,
            wheelbase=float(self.get_parameter("wheelbase").value),
            max_steering=math.radians(float(self.get_parameter("max_steering_deg").value)),
            steering_rate_limit=self.steering_rate_limit,
            **({
                "k": float(self.get_parameter("k").value),
                "k_soft": float(self.get_parameter("k_soft").value),
                "use_feedforward": bool(self.get_parameter("feedforward").value),
            } if controller_name == "stanley" else {}),
        )

        self.speed_profile = SpeedProfile(
            target_speed=float(self.get_parameter("target_speed").value),
            min_speed=float(self.get_parameter("min_speed").value),
            a_lat_max=float(self.get_parameter("a_lat_max").value),
        )

        scenario = str(self.get_parameter("reference_scenario").value)
        self.path = build_scenario(scenario)
        self.wait_for_path = bool(self.get_parameter("wait_for_path").value)
        self.get_logger().info(
            f"已载入内置参考路径 scenario={scenario}，点数={len(self.path)}，"
            f"长度={self.path.total_length:.1f} m"
        )

        self.state = None          # (x, y, yaw)
        self.speed = self.speed_profile.target_speed
        self.dt = 1.0 / float(self.get_parameter("control_rate_hz").value)

        self.publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publisher_steering = self.create_publisher(Float64, "/stanley/steering", 10)
        self.publisher_error = self.create_publisher(Float64MultiArray, "/stanley/tracking_error", 10)

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/reference_path", self.on_path, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("Stanley 跟踪节点已启动，等待 /odom ...")

    # ------------------------------------------------------------------ 回调
    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = yaw_from_quaternion(orientation.x, orientation.y,
                                  orientation.z, orientation.w)
        self.state = np.array([position.x, position.y, yaw])
        velocity = message.twist.twist.linear
        self.speed = float(np.hypot(velocity.x, velocity.y))
        if self.speed < 1e-3:
            # 速度观测为 0 时用目标速度兜底，避免原地不动导致前馈项失效
            self.speed = self.speed_profile.target_speed

    def on_path(self, message: Path) -> None:
        points = np.array([[pose.pose.position.x, pose.pose.position.y]
                           for pose in message.poses], dtype=float)
        if points.shape[0] < 2:
            self.get_logger().warn("收到的 /reference_path 点数不足 2，已忽略")
            return
        self.path = ReferencePath(points)
        self.wait_for_path = False
        self.get_logger().info(f"已更新参考路径：{points.shape[0]} 个点，"
                               f"{self.path.total_length:.1f} m")

    def on_timer(self) -> None:
        if self.state is None:
            return
        if self.wait_for_path:
            return

        projection = self.path.project(self.state[:2])
        speed_target = self.speed_profile.speed_for(self.path, projection)
        speed = 0.5 * (self.speed + speed_target)
        command = self.controller.compute(self.state, speed, self.path, projection, self.dt)

        twist = Twist()
        twist.linear.x = float(command.speed)
        twist.angular.z = float(command.yaw_rate)
        self.publisher_cmd.publish(twist)

        steering_message = Float64()
        steering_message.data = float(command.steering)
        self.publisher_steering.publish(steering_message)

        error_message = Float64MultiArray()
        error_message.data = [float(command.cross_track_error),
                              float(command.heading_error),
                              float(command.steering),
                              float(command.speed)]
        self.publisher_error.publish(error_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StanleyTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发零速度，避免小车失控
        twist = Twist()
        node.publisher_cmd.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
