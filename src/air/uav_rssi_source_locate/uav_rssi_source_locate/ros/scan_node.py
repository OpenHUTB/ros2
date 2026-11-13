#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RSSI measurement node: pair own odometry with incoming RSSI readings,
run trilateration once enough samples are in, publish the fix.

Single-line summary: thin ROS shell around core.locator — the node only
keeps the sample bookkeeping and the periodic solve; the geometry, the
path-loss model and the damping all live in core.
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

from uav_rssi_source_locate.core.locator import localize
from uav_rssi_source_locate.core.rssi import RssiCfg


class ScanNode(Node):
    """订阅 /uav/odom 与 RSSI 读数，攒够样本后发布定位结果。

    只收有限值：底噪以下的读数在 core 里就是 nan，节点直接丢弃——把
    nan 塞进最小二乘会让整个解变成 nan，而且看不出是哪一条坏样本。
    """

    def __init__(self):
        super().__init__("uav_rssi_source_locate_node")
        self.declare_parameter("rssi0", -40.0)
        self.declare_parameter("n", 2.2)
        self.declare_parameter("d0", 1.0)
        self.declare_parameter("floor", -76.0)
        self.declare_parameter("quant_db", 0.5)
        self.declare_parameter("beacon_z", 0.5)
        self.declare_parameter("min_samples", 8)
        self.declare_parameter("max_samples", 40)
        self.declare_parameter("solve_period_s", 1.0)
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("rssi_topic", "/rssi")
        self.declare_parameter("estimate_topic",
                               "/uav_rssi_source_locate/estimate")

        self.rssi_cfg = RssiCfg(
            rssi0=float(self.get_parameter("rssi0").value),
            n=float(self.get_parameter("n").value),
            d0=float(self.get_parameter("d0").value),
            floor=float(self.get_parameter("floor").value),
            quant_db=float(self.get_parameter("quant_db").value))
        self.beacon_z = float(self.get_parameter("beacon_z").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.max_samples = int(self.get_parameter("max_samples").value)
        self.estimate = None

        self.pub = self.create_publisher(
            String, self.get_parameter("estimate_topic").value, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._on_odom, 10)
        self.create_subscription(
            Float32, self.get_parameter("rssi_topic").value,
            self._on_rssi, 10)
        self.create_timer(float(self.get_parameter("solve_period_s").value),
                          self._solve)

        self.pos = None
        self.samples = []      # [(3D 位置, RSSI)]

    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z], dtype=float)

    def _on_rssi(self, msg: Float32) -> None:
        if self.pos is None or not np.isfinite(msg.data):
            return
        self.samples.append((self.pos.copy(), float(msg.data)))
        if len(self.samples) > self.max_samples:
            del self.samples[0]

    def _solve(self) -> None:
        if len(self.samples) < self.min_samples:
            return
        out = localize(self.samples, self.rssi_cfg, init=self.estimate,
                       beacon_z=self.beacon_z)
        if not out["feasible"]:
            self.get_logger().debug("solve skipped: %s" % out["reason"])
            return
        self.estimate = out["estimate"]
        self.pub.publish(String(data=json.dumps(dict(
            x=round(float(self.estimate[0]), 3),
            y=round(float(self.estimate[1]), 3),
            residual_m=out["residual_m"],
            n_samples=out["n_samples"]), ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
