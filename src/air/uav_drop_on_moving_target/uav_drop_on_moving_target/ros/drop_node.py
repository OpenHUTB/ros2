#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""薄壳节点：订阅机体与车队里程计，按 evaluate_release 判定后发投放指令。

判据与闭环用的是同一个函数（core.release.evaluate_release），节点只负责
把里程计喂进去、把结论发出去。车队未来位置用当前速度外推（直线假设），
航迹已知时应在 launch 里换成带 waypoints 的模型。
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ..core.release import DropCfg, evaluate_release
from ..core.vehicle import Vehicle

#: 外推窗口（s）：超过这个时长的预测不再可信，节点只在这个范围内判定
PREDICT_HORIZON_S = 30.0


class DropOnMovingTargetNode(Node):
    """把"什么时候松手"做成一条 ROS 话题输出。"""

    def __init__(self):
        super().__init__("uav_drop_on_moving_target_node")
        self.declare_parameter("alt_m", 30.0)
        self.declare_parameter("accept_radius_m", 6.0)
        self.declare_parameter("wind_x", 0.0)
        self.declare_parameter("wind_y", 0.0)
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("convoy_topic", "/convoy/odom")
        self.declare_parameter("release_topic", "/uav_drop_on_moving_target/release")

        self.cfg = DropCfg(
            alt_m=float(self.get_parameter("alt_m").value),
            accept_radius_m=float(self.get_parameter("accept_radius_m").value),
            wind=(float(self.get_parameter("wind_x").value),
                  float(self.get_parameter("wind_y").value)))

        self._uav = None
        self._convoy = None
        self._released = False
        self._t0 = self.get_clock().now()

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_uav, 10)
        self.create_subscription(
            Odometry, self.get_parameter("convoy_topic").value, self._on_convoy, 10)
        self.pub = self.create_publisher(
            String, self.get_parameter("release_topic").value, 10)
        self.get_logger().info("drop_on_moving_target ready")

    # ------------------------------------------------------------------ #
    def _on_uav(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._uav = (np.array([p.x, p.y, p.z], dtype=float),
                     np.array([v.x, v.y, v.z], dtype=float))
        self._maybe_release()

    def _on_convoy(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self._convoy = (np.array([p.x, p.y], dtype=float),
                        np.array([v.x, v.y], dtype=float))

    def _maybe_release(self) -> None:
        if self._released or self._uav is None or self._convoy is None:
            return
        pos, vel = self._uav
        cpos, cvel = self._convoy
        speed = float(np.linalg.norm(cvel))
        if speed < 0.5:
            return
        # 直线外推的车队模型：只在预测窗口内有意义
        vehicle = Vehicle(
            "convoy", (tuple(cpos), tuple(cpos + cvel / speed * speed * PREDICT_HORIZON_S)),
            speed)
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        verdict = evaluate_release(pos, vel, t, vehicle, self.cfg)
        if not verdict["ok"]:
            return
        self._released = True
        msg = String()
        msg.data = json.dumps(dict(
            t_release=round(t, 2),
            release_pos=[round(float(x), 2) for x in pos],
            release_vel=[round(float(x), 2) for x in vel],
            t_fall_s=round(float(verdict["t_fall"]), 3),
            impact_m=[round(float(x), 2) for x in verdict["impact_xy"]],
            convoy_at_impact_m=[round(float(x), 2) for x in verdict["target_xy"]],
            miss_m=round(float(verdict["miss_m"]), 2)))
        self.pub.publish(msg)
        self.get_logger().info("release fired: miss %.2f m" % verdict["miss_m"])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropOnMovingTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
