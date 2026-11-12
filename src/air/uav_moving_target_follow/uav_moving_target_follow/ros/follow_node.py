#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Follow decision node: subscribe own odom + target odom, publish the
standoff reference the UAV should fly.

Single-line summary: thin ROS shell around core.follow — each pair of
odometry samples is turned into a standoff reference (position/velocity/
yaw) published as one JSON message; all geometry lives in core.follow.
"""
from __future__ import annotations

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_moving_target_follow.core.follow import FollowCfg, follow_reference


class FollowNode(Node):
    """订阅 /uav/odom 与 /target/odom，发布 standoff 参考点。"""

    def __init__(self):
        super().__init__("uav_moving_target_follow_node")
        self.declare_parameter("standoff_m", 12.0)
        self.declare_parameter("alt_m", 10.0)
        self.declare_parameter("cruise_speed", 9.0)
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("target_odom_topic", "/target/odom")
        self.declare_parameter("setpoint_topic",
                               "/uav_moving_target_follow/setpoint")
        self.cfg = FollowCfg(
            standoff_m=float(self.get_parameter("standoff_m").value),
            alt_m=float(self.get_parameter("alt_m").value),
            cruise_speed=float(self.get_parameter("cruise_speed").value))

        self.pub = self.create_publisher(
            String, self.get_parameter("setpoint_topic").value, 10)
        self.sub_uav = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_uav, 10)
        self.sub_tgt = self.create_subscription(
            Odometry, self.get_parameter("target_odom_topic").value,
            self._on_target, 10)

        self.pos = (0.0, 0.0, 0.0)
        self.tgt_pos = None            # (x, y)
        self.tgt_vel = None            # (vx, vy)
        self._prev_dir = None

    # ------------------------------------------------------------------ #
    def _on_uav(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        self._publish()

    def _on_target(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.tgt_pos = (p.x, p.y)
        self.tgt_vel = (v.x, v.y)
        self._publish()

    def _publish(self) -> None:
        if self.tgt_pos is None or self.tgt_vel is None:
            return
        p_ref, v_ref, d_off = follow_reference(
            self.tgt_pos, self.tgt_vel, self.cfg, self._prev_dir)
        self._prev_dir = d_off
        yaw_d = math.atan2(self.tgt_pos[1] - self.pos[1],
                           self.tgt_pos[0] - self.pos[0])
        msg = String(data=json.dumps(dict(
            p_ref=dict(x=round(float(p_ref[0]), 3),
                       y=round(float(p_ref[1]), 3),
                       z=round(float(p_ref[2]), 3)),
            v_ref=dict(x=round(float(v_ref[0]), 3),
                       y=round(float(v_ref[1]), 3)),
            yaw_d=round(yaw_d, 3),
            standoff_m=self.cfg.standoff_m, alt_m=self.cfg.alt_m),
            ensure_ascii=False))
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
