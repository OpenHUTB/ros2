#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Turtlebot3 激光雷达避障节点。

对应文档：docs/ground/tb3_lidar_obstacle.md

用法：
    ros2 run tb3_lidar_obstacle obstacle_avoid
或直接：
    python3 main.py

相比初版修正了两处：
  1. 按 angle_min/angle_increment 反查 0° 方向，而不是硬取 ranges[0]
     （ranges[0] 对应扫描起始角，不一定是正前方）
  2. 无效值填充为 range_max * 2，而不是写死 10.0，
     让"无回波"的语义与"阈值"解耦
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__("obstacle_avoid")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Gazebo 默认以 BEST_EFFORT 发布雷达数据，订阅端必须对齐
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, scan_qos
        )

        self.twist = Twist()
        self.safe_dist = 0.5      # m
        self.forward_speed = 0.2  # m/s
        self.turn_speed = 0.3     # rad/s

    def front_distance(self, msg: LaserScan) -> float:
        """按角度反查正前方距离，避免 ranges[0] 的方向不确定性。"""
        span = msg.angle_min + msg.angle_increment * len(msg.ranges)
        if msg.angle_increment == 0.0:
            return msg.range_max * 2.0

        idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        # 处理跨 0/2π 的情况
        if idx < 0:
            idx += len(msg.ranges)
        idx %= len(msg.ranges)

        d = msg.ranges[idx]
        if math.isinf(d) or math.isnan(d) or d < msg.range_min:
            return msg.range_max * 2.0
        return d

    def scan_callback(self, msg: LaserScan):
        front = self.front_distance(msg)
        self.get_logger().info(f"前方距离: {front:.2f} m")

        if front < self.safe_dist:
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.turn_speed
        else:
            self.twist.linear.x = self.forward_speed
            self.twist.angular.z = 0.0

        self.pub.publish(self.twist)


def main():
    rclpy.init()
    node = ObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
