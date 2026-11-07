#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串级 PID 控制节点：订阅 /odom，发布 /cmd_motors。"""

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .config_loader import build_params, load_config, default_params_path
from .controller import CascadeController, WaypointManager
from .dynamics import Quadrotor
from .geometry import quat_to_euler


class ControllerNode(Node):
    def __init__(self):
        super().__init__("quad_controller")
        self.declare_parameter("params_file", default_params_path())
        self.declare_parameter("rate", 200.0)

        cfg = load_config(self.get_parameter("params_file").value)
        self.qp, self.wp, self.gains, self.cfg = build_params(cfg)
        self.quad = Quadrotor(self.qp, self.wp)

        wp_cfg = self.cfg["waypoints"]
        acceptance = float(wp_cfg["acceptance_radius"])
        hover_time = float(wp_cfg["hover_time"])
        wps = [list(w) for w in wp_cfg["points"]]
        self.wp_mgr = WaypointManager(wps, acceptance, hover_time)

        yaw_control = bool(wp_cfg.get("yaw_control", False))
        self.ctrl = CascadeController(self.qp, self.gains, self.quad._mixer,
                                      yaw_control=yaw_control)

        self.rate = float(self.get_parameter("rate").value)
        self.odom: Optional[Odometry] = None
        self.manual_target: Optional[np.ndarray] = None

        self.cmd_pub = self.create_publisher(Float32MultiArray, "/cmd_motors", 10)
        self.target_pub = self.create_publisher(Point, "/target", 10)
        self.info_pub = self.create_publisher(String, "/nav_info", 10)

        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        self.timer = self.create_timer(1.0 / self.rate, self.update)
        self.last_t = None
        self.get_logger().info("串级 PID 控制节点已启动：%.0f Hz，%d 个航点"
                               % (self.rate, len(wps)))

    def on_odom(self, msg: Odometry):
        self.odom = msg

    def on_goal(self, msg: PoseStamped):
        self.manual_target = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def update(self):
        if self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        self.last_t = now
        dt = min(dt, 0.1)

        p = self.odom.pose.pose.position
        o = self.odom.pose.pose.orientation
        v = self.odom.twist.twist.linear
        w = self.odom.twist.twist.angular
        pos = np.array([p.x, p.y, p.z])
        vel = np.array([v.x, v.y, v.z])
        quat = np.array([o.w, o.x, o.y, o.z])
        omega = np.array([w.x, w.y, w.z])

        if self.manual_target is not None:
            target = self.manual_target
        else:
            target, _ = self.wp_mgr.update(pos, dt)

        omega_cmd = self.ctrl.compute(pos, vel, quat, omega, target, dt)
        msg = Float32MultiArray()
        msg.data = [float(x) for x in omega_cmd]
        self.cmd_pub.publish(msg)

        tp = Point()
        tp.x = float(target[0]); tp.y = float(target[1]); tp.z = float(target[2])
        self.target_pub.publish(tp)

        err = np.linalg.norm(pos - target)
        if int(now) % 5 == 0:
            self.info_pub.publish(String(
                data="target=(%.2f,%.2f,%.2f) err=%.2fm" % (
                    target[0], target[1], target[2], err)))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
