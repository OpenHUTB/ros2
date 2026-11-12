#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四旋翼仿真节点：积分 6-DOF 动力学，发布 /odom 与 /ground_truth。"""

import math
import os
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .config_loader import build_params, load_config, default_params_path
from .dynamics import Quadrotor, State
from .geometry import quat_to_euler


class SimNode(Node):
    def __init__(self):
        super().__init__("quad_sim")
        self.declare_parameter("params_file", default_params_path())
        self.declare_parameter("dt", 0.002)
        self.declare_parameter("publish_rate", 200.0)

        cfg = load_config(self.get_parameter("params_file").value)
        self.qp, self.wp, self.gains, self.cfg = build_params(cfg)
        self.quad = Quadrotor(self.qp, self.wp)

        self.dt = float(self.get_parameter("dt").value)
        self.pub_rate = float(self.get_parameter("publish_rate").value)

        init = self.cfg["initial_pose"]
        self.state = State(
            pos=np.array(init["position"], float),
            quat=np.array([math.cos(init["yaw"]/2), 0, 0, math.sin(init["yaw"]/2)]),
        )
        hover_omega = math.sqrt(self.qp.mass * self.qp.gravity / (4 * self.qp.k_f))
        self.state.omega_cmd = np.full(4, hover_omega)

        self.omega_cmd = self.state.omega_cmd.copy()
        self.t = 0.0

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth", 10)
        self.wind_pub = self.create_publisher(Vector3Stamped, "/wind", 10)
        self.create_subscription(Float32MultiArray, "/cmd_motors", self.on_cmd, 10)

        self.timer = self.create_timer(self.dt, self.step)
        self.pub_counter = 0
        self.get_logger().info("四旋翼仿真节点已启动：mass=%.2f, arm=%.2f, dt=%.4f"
                               % (self.qp.mass, self.qp.arm, self.dt))

    def on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.omega_cmd = np.array(msg.data[:4], dtype=float)

    def step(self):
        self.state, wind = self.quad.step(self.state, self.omega_cmd, self.dt, self.t)
        self.t += self.dt

        # 按 publish_rate 发布
        self.pub_counter += 1
        skip = max(1, int(round(1.0 / (self.dt * self.pub_rate))))
        if self.pub_counter % skip == 0:
            self._publish(wind)

    def _publish(self, wind: np.ndarray):
        stamp = self.get_clock().now().to_msg()
        q = self.state.quat
        roll, pitch, yaw = quat_to_euler(q)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.state.pos[0])
        odom.pose.pose.position.y = float(self.state.pos[1])
        odom.pose.pose.position.z = float(self.state.pos[2])
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(self.state.vel[0])
        odom.twist.twist.linear.y = float(self.state.vel[1])
        odom.twist.twist.linear.z = float(self.state.vel[2])
        odom.twist.twist.angular.x = float(self.state.omega[0])
        odom.twist.twist.angular.y = float(self.state.omega[1])
        odom.twist.twist.angular.z = float(self.state.omega[2])
        self.odom_pub.publish(odom)
        self.gt_pub.publish(odom)

        w = Vector3Stamped()
        w.header.stamp = stamp
        w.header.frame_id = "world"
        w.vector = Vector3(x=float(wind[0]), y=float(wind[1]), z=float(wind[2]))
        self.wind_pub.publish(w)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
