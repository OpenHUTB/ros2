#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drop decision node: subscribe odom, publish drop command in the window.

Single-line summary: thin ROS shell around core.trigger — each odom sample
is evaluated against the analytic release window; entering it publishes one
JSON drop command with the predicted impact point.

Parameters (config/uav_emergency_drop.yaml)::

    seg_a / seg_b       approach leg endpoints [x, y]
    target_x, target_y  true demand point
    drop_h, drop_speed  release conditions
    wind_x, wind_y      horizontal wind (m/s)

Publishes::

    /uav_emergency_drop/drop  std_msgs/String  JSON drop command
"""
from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_emergency_drop.core.ballistics import make_release_plan
from uav_emergency_drop.core.trigger import evaluate_drop


class DropNode(Node):
    def __init__(self) -> None:
        super().__init__("uav_emergency_drop_node")
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("seg_a", [0.0, 0.0])
        self.declare_parameter("seg_b", [200.0, 0.0])
        self.declare_parameter("target", [200.0, 0.0])
        self.declare_parameter("drop_h", 30.0)
        self.declare_parameter("drop_speed", 8.0)
        self.declare_parameter("wind", [0.0, 0.0])
        self.declare_parameter("drop_topic", "/uav_emergency_drop/drop")

        self.seg_a = [float(v) for v in self.get_parameter("seg_a").value]
        self.seg_b = [float(v) for v in self.get_parameter("seg_b").value]
        self.target = [float(v) for v in self.get_parameter("target").value]
        wind = [float(v) for v in self.get_parameter("wind").value]
        self.plan = make_release_plan(
            float(self.get_parameter("drop_h").value),
            float(self.get_parameter("drop_speed").value), wind)

        self.released = False
        self.pub = self.create_publisher(
            String, str(self.get_parameter("drop_topic").value), 10)
        self.create_subscription(Odometry,
                                 str(self.get_parameter("odom_topic").value),
                                 self._on_odom, 10)
        from uav_emergency_drop.core.trigger import check_leg
        pre = check_leg(self.seg_a, self.seg_b, self.target, self.plan)
        self.get_logger().info(
            "drop node up: target (%.1f, %.1f) m, lead %.1f m, sigma %.1f m"
            % (self.target[0], self.target[1], self.plan.lead_m,
               self.plan.sigma))
        self.get_logger().info("leg check: %s" % pre.as_dict())

    def _on_odom(self, msg: Odometry) -> None:
        if self.released:
            return
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        verdict = evaluate_drop((p.x, p.y), (v.x, v.y),
                                self.seg_a, self.seg_b, self.target,
                                self.plan, self.released)
        if verdict.release_now:
            self.released = True
            cmd = dict(command="drop", target_m=self.target,
                       predicted_impact_m=verdict.as_dict()["predicted_impact_m"],
                       lead_m=verdict.window.lead_m,
                       sigma_m=self.plan.sigma)
            self.pub.publish(String(data=json.dumps(cmd)))
            self.get_logger().warn(
                "DROP issued: predicted impact (%.2f, %.2f) m"
                % tuple(cmd["predicted_impact_m"]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
