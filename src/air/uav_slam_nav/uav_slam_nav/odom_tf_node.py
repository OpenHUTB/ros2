#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Noetic bridge 发过来的 /uav/odom 转成 odom -> base_link 的 TF。

标准 ROS 导航栈的 TF 树是 map -> odom -> base_link：
  - slam_toolbox 发 map -> odom
  - 本节点把 /uav/odom 发成 odom -> base_link
这样 point cloud 投影、nav2 costmap 都能走标准 TF，不用自己解位姿。
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, "/uav/odom", self.cb, 10)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.get_parameter("odom_frame").value
        t.child_frame_id = self.get_parameter("base_frame").value
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
