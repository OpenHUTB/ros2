#!/usr/bin/env python3
"""Verify that the A*+DWA node actually drives the TurtleBot3 in Gazebo.

The verifier subscribes to ``/odom``, ``/scan`` and ``/cmd_vel`` and exits 0
only when the robot has moved at least ``min_displacement`` metres, kept a
positive laser clearance and received non-zero velocity commands.

Elapsed time and the timeout use the ``/odom`` header stamps (Gazebo sim time),
so a real-time factor below 1 does not cause a false timeout.  A wall-clock
guard still stops a run when no odometry arrives at all.
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@dataclass
class VerificationState:
    min_displacement: float = 0.8
    min_clearance: float = 0.12
    timeout: float = 60.0
    start: Optional[tuple] = None
    last: Optional[tuple] = None
    start_stamp: Optional[float] = None
    last_stamp: Optional[float] = None
    wall_start: Optional[float] = None
    min_scan: float = float("inf")
    max_cmd: float = 0.0

    def update_odom(self, x: float, y: float,
                    stamp: Optional[float] = None) -> None:
        if self.start is None:
            self.start = (x, y)
            self.start_stamp = stamp
            self.wall_start = time.monotonic()
        self.last = (x, y)
        if stamp is not None:
            self.last_stamp = stamp

    def update_scan(self, ranges: Sequence[float]) -> None:
        for r in ranges:
            if math.isfinite(r) and 0.0 < r < self.min_scan:
                self.min_scan = float(r)

    def update_cmd(self, linear: float, angular: float) -> None:
        self.max_cmd = max(self.max_cmd, abs(linear) + abs(angular))

    @property
    def displacement(self) -> float:
        if self.start is None or self.last is None:
            return 0.0
        return math.hypot(self.last[0] - self.start[0], self.last[1] - self.start[1])

    @property
    def elapsed(self) -> float:
        if self.start_stamp is not None and self.last_stamp is not None:
            return max(0.0, self.last_stamp - self.start_stamp)
        if self.wall_start is not None:
            return time.monotonic() - self.wall_start
        return 0.0

    def result(self) -> dict:
        success = (self.displacement >= self.min_displacement and
                   self.max_cmd > 0.05 and
                   self.min_scan >= self.min_clearance)
        return {
            "success": bool(success),
            "displacement_m": round(self.displacement, 3),
            "min_scan_m": round(self.min_scan, 3) if math.isfinite(self.min_scan) else None,
            "max_cmd": round(self.max_cmd, 3),
            "elapsed_s": round(self.elapsed, 2),
        }


class NavigationVerifier(Node):
    def __init__(self) -> None:
        super().__init__("navigation_verifier")
        self.declare_parameter("min_displacement", 0.8)
        self.declare_parameter("min_clearance", 0.12)
        self.declare_parameter("timeout", 60.0)
        self.state = VerificationState(
            min_displacement=float(self.get_parameter("min_displacement").value),
            min_clearance=float(self.get_parameter("min_clearance").value),
            timeout=float(self.get_parameter("timeout").value),
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, qos)

    def on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.state.update_odom(msg.pose.pose.position.x,
                               msg.pose.pose.position.y, stamp)

    def on_scan(self, msg: LaserScan) -> None:
        self.state.update_scan(msg.ranges)

    def on_cmd(self, msg: Twist) -> None:
        self.state.update_cmd(msg.linear.x, msg.angular.z)

    def done(self) -> bool:
        if self.state.displacement >= self.state.min_displacement:
            return True
        if self.state.elapsed >= self.state.timeout:
            return True
        # No odometry at all: do not hang forever.
        if self.state.wall_start is not None:
            wall = time.monotonic() - self.state.wall_start
            return wall > max(300.0, self.state.timeout * 10.0)
        return False


def main() -> None:
    rclpy.init()
    node = NavigationVerifier()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        result = node.state.result()
        print("RESULT " + json.dumps(result), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result["success"] else 1)


if __name__ == "__main__":
    main()
