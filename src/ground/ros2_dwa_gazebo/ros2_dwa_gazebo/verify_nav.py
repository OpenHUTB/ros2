#!/usr/bin/env python3
"""Verify that the A*+DWA node actually drives the TurtleBot3 in Gazebo.

The verifier subscribes to ``/odom``, ``/scan`` and ``/cmd_vel`` and exits 0
when the robot reaches the goal (preferred), moves at least
``min_displacement`` metres, keeps a positive laser clearance and receives
non-zero velocity commands.

Elapsed time and the timeout use the ``/odom`` header stamps (Gazebo sim time),
so a real-time factor below 1 does not cause a false timeout.  The wall-clock
guard starts at node construction, so a run with no odometry at all still exits.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
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
    timeout: float = 120.0
    goal: tuple = (2.5, 0.0)
    goal_tolerance: float = 0.25
    start: Optional[tuple] = None
    last: Optional[tuple] = None
    start_stamp: Optional[float] = None
    last_stamp: Optional[float] = None
    wall_start: Optional[float] = field(default=None)
    min_scan: float = float("inf")
    max_cmd: float = 0.0
    goal_reached: bool = False

    def __post_init__(self) -> None:
        # Start the wall guard immediately: a run with no odom must still exit.
        self.wall_start = time.monotonic()

    def update_odom(self, x: float, y: float,
                    stamp: Optional[float] = None) -> None:
        if self.start is None:
            self.start = (x, y)
            self.start_stamp = stamp
        self.last = (x, y)
        if stamp is not None:
            self.last_stamp = stamp
        if self.goal_distance is not None and self.goal_distance <= self.goal_tolerance:
            self.goal_reached = True

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
    def goal_distance(self) -> Optional[float]:
        if self.last is None:
            return None
        return math.hypot(self.last[0] - self.goal[0], self.last[1] - self.goal[1])

    @property
    def elapsed(self) -> float:
        if self.start_stamp is not None and self.last_stamp is not None:
            return max(0.0, self.last_stamp - self.start_stamp)
        if self.wall_start is not None:
            return time.monotonic() - self.wall_start
        return 0.0

    @property
    def wall_age(self) -> float:
        return time.monotonic() - self.wall_start if self.wall_start else 0.0

    def result(self) -> dict:
        success = (self.goal_reached or self.displacement >= self.min_displacement) and \
                  self.max_cmd > 0.05 and self.min_scan >= self.min_clearance
        return {
            "success": bool(success),
            "displacement_m": round(self.displacement, 3),
            "min_scan_m": round(self.min_scan, 3) if math.isfinite(self.min_scan) else None,
            "max_cmd": round(self.max_cmd, 3),
            "elapsed_s": round(self.elapsed, 2),
            "goal_reached": bool(self.goal_reached),
        }


class NavigationVerifier(Node):
    def __init__(self) -> None:
        super().__init__("navigation_verifier")
        self.declare_parameter("min_displacement", 0.8)
        self.declare_parameter("min_clearance", 0.12)
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("goal_x", 2.5)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_tolerance", 0.25)
        self.state = VerificationState(
            min_displacement=float(self.get_parameter("min_displacement").value),
            min_clearance=float(self.get_parameter("min_clearance").value),
            timeout=float(self.get_parameter("timeout").value),
            goal=(float(self.get_parameter("goal_x").value),
                  float(self.get_parameter("goal_y").value)),
            goal_tolerance=float(self.get_parameter("goal_tolerance").value),
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
        if self.state.goal_reached:
            return True
        if self.state.elapsed >= self.state.timeout:
            return True
        # No odometry at all: the wall clock still terminates the run.
        return self.state.wall_age > max(300.0, self.state.timeout * 10.0)


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
