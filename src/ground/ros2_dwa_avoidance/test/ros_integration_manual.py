#!/usr/bin/env python3
"""Closed-loop ROS 2 integration test for the DWA node.

The harness plays the role of both the robot and the world:

* subscribes ``/cmd_vel`` and integrates a differential-drive model;
* ray-casts the same corridor world used by ``sim.py`` and publishes the
  resulting ``sensor_msgs/LaserScan`` on ``/scan``;
* publishes ``nav_msgs/Odometry`` on ``/odom``.

Run it while ``main_dwa`` is running; it exits 0 when the robot reaches the
goal without collision.
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "ros2_dwa_avoidance"))
import sim  # noqa: E402


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("dwa_integration_harness")
        # A 2.5 m corridor: this test validates the ROS topic/control chain,
        # not the tightest offline benchmark scenario (1.9 m).
        self.scenario = sim.Scenario(
            "ros_corridor", (-0.5, 0.0, 0.0), (11.5, 0.0),
            [sim.Circle(3.0, 0.35, 0.22), sim.Circle(6.0, -0.35, 0.22),
             sim.Circle(9.0, 0.35, 0.22)],
            [sim.Segment(-2.0, 1.25, 14.0, 1.25),
             sim.Segment(-2.0, -1.25, 14.0, -1.25)],
            60.0, "ROS integration corridor")
        self.params = sim.build_params()
        self.dt = 0.1
        self.x, self.y, self.yaw = self.scenario.start
        self.v = 0.0
        self.w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.path = 0.0
        self.min_clear = float("inf")
        self.start_time = time.monotonic()
        self.steps = 0
        self.finished = False
        self.success = False
        self.collision = False

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("harness ready: corridor world, goal (11.5, 0.0)")

    def on_cmd(self, msg: Twist) -> None:
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def on_timer(self) -> None:
        if self.finished:
            return
        p = self.params
        # Actuator limits, identical to sim.simulate().
        self.v += max(-p.max_acc_v * self.dt, min(p.max_acc_v * self.dt, self.cmd_v - self.v))
        self.w += max(-p.max_acc_w * self.dt, min(p.max_acc_w * self.dt, self.cmd_w - self.w))

        if abs(self.w) < 1e-6:
            nx = self.x + self.v * self.dt * math.cos(self.yaw)
            ny = self.y + self.v * self.dt * math.sin(self.yaw)
        else:
            r = self.v / self.w
            nx = self.x + r * (math.sin(self.yaw + self.w * self.dt) - math.sin(self.yaw))
            ny = self.y - r * (math.cos(self.yaw + self.w * self.dt) - math.cos(self.yaw))
        self.yaw = sim.wrap_angle(self.yaw + self.w * self.dt)
        self.path += math.hypot(nx - self.x, ny - self.y)
        self.x, self.y = nx, ny

        ranges, _ = sim._raycast((self.x, self.y), self.yaw,
                                 self.scenario.circles, self.scenario.segments,
                                 180, 6.0)
        clear = sim.world_clearance((self.x, self.y), self.scenario.circles,
                                    self.scenario.segments)
        self.min_clear = min(self.min_clear, clear)

        stamp = self.get_clock().now().to_msg()

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / len(ranges)
        scan.time_increment = 0.0
        scan.scan_time = self.dt
        scan.range_min = 0.05
        scan.range_max = 6.0
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        self.steps += 1
        self.last_cmd = (self.cmd_v, self.cmd_w)
        if self.steps % 50 == 0:
            self.get_logger().info(
                f"t={self.steps * self.dt:.1f}s pose=({self.x:.2f},{self.y:.2f}) "
                f"cmd=({self.cmd_v:.2f},{self.cmd_w:.2f}) clear={clear:.2f}")

        goal_dist = math.hypot(self.scenario.goal[0] - self.x,
                               self.scenario.goal[1] - self.y)
        if clear < 0.0:
            self.collision = True
            self.finished = True
        elif goal_dist < 0.25:
            self.success = True
            self.finished = True
        elif self.steps * self.dt > 60.0:
            self.finished = True

        if self.finished:
            elapsed = self.steps * self.dt
            print(f"RESULT success={self.success} collision={self.collision} "
                  f"time={elapsed:.1f} path={self.path:.2f} "
                  f"min_clear={self.min_clear:.3f} "
                  f"final=({self.x:.2f},{self.y:.2f})", flush=True)


def main() -> None:
    rclpy.init()
    node = Harness()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if node.success and not node.collision else 1)


if __name__ == "__main__":
    main()
