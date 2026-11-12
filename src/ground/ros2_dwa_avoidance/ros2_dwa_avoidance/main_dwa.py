#!/usr/bin/env python3
"""ROS 2 Humble node: A* global route + DWA local obstacle avoidance.

Subscribes
    /scan       sensor_msgs/LaserScan
    /odom       nav_msgs/Odometry
    /goal_pose  geometry_msgs/PoseStamped  (optional, overrides goal_x/goal_y)

Publishes
    /cmd_vel    geometry_msgs/Twist

The node keeps a world-frame occupancy grid built from laser points.  A* is
re-run on that grid every ``replan_period`` seconds and the DWA layer tracks
the resulting route while still rejecting trajectories that collide with the
latest scan.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from .dwa_core import DWAParams, RobotState, scan_points_from_ranges
from .hybrid_planner import AStarDWAController


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DwaAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("dwa_avoidance")

        # ---- parameters ---------------------------------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("use_goal_pose_topic", True)

        self.declare_parameter("max_v", 0.80)
        self.declare_parameter("max_w", 1.80)
        self.declare_parameter("max_acc_v", 1.20)
        self.declare_parameter("max_acc_w", 3.00)
        self.declare_parameter("predict_time", 1.50)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("safety_margin", 0.08)
        self.declare_parameter("heading_weight", 0.60)
        self.declare_parameter("velocity_weight", 1.50)
        self.declare_parameter("clearance_weight", 0.60)
        self.declare_parameter("clearance_norm", 0.70)
        self.declare_parameter("goal_tolerance", 0.25)

        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("replan_period", 1.00)
        self.declare_parameter("lookahead", 1.20)
        self.declare_parameter("map_half_size", 10.00)
        self.declare_parameter("data_timeout", 0.50)
        self.declare_parameter("max_scan_points", 120)
        self.declare_parameter("laser_x", 0.0)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("publish_tf", True)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.replan_period = float(self.get_parameter("replan_period").value)
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.map_half = float(self.get_parameter("map_half_size").value)
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        self.max_scan_points = int(self.get_parameter("max_scan_points").value)
        self.laser_x = float(self.get_parameter("laser_x").value)
        self.laser_y = float(self.get_parameter("laser_y").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        p = self._build_params()

        self.state = RobotState()
        self.last_odom_time: float | None = None
        self.last_scan_time: float | None = None
        self.scan_msg: LaserScan | None = None
        self.goal = (self.goal_x, self.goal_y)

        bounds = (self.state.x - self.map_half, self.state.y - self.map_half,
                  self.state.x + self.map_half, self.state.y + self.map_half)
        self.controller = AStarDWAController(
            p, bounds, replan_period=self.replan_period,
            lookahead=self.lookahead, goal_tolerance=p.goal_tolerance)
        self._window_center = (self.state.x, self.state.y)

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value),
                                 self.on_scan, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self.on_odom, 10)
        if bool(self.get_parameter("use_goal_pose_topic").value):
            self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self.on_timer)
        self._last_log = 0.0
        self._warned_no_goal = False
        self.get_logger().info(
            f"dwa_avoidance ready: goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}) "
            f"v_max={p.max_v:.2f} m/s")

    # ------------------------------------------------------------------
    def _build_params(self) -> DWAParams:
        def get(name: str):
            return self.get_parameter(name).value

        return DWAParams(
            max_v=float(get("max_v")),
            max_w=float(get("max_w")),
            max_acc_v=float(get("max_acc_v")),
            max_acc_w=float(get("max_acc_w")),
            predict_time=float(get("predict_time")),
            robot_radius=float(get("robot_radius")),
            safety_margin=float(get("safety_margin")),
            heading_weight=float(get("heading_weight")),
            velocity_weight=float(get("velocity_weight")),
            clearance_weight=float(get("clearance_weight")),
            clearance_norm=float(get("clearance_norm")),
            goal_tolerance=float(get("goal_tolerance")),
            max_scan_points=int(get("max_scan_points")),
            dt=1.0 / max(float(get("control_rate")), 1.0),
        )

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        self.state.x = pos.x
        self.state.y = pos.y
        self.state.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.state.v = msg.twist.twist.linear.x
        self.state.w = msg.twist.twist.angular.z
        self.last_odom_time = time.monotonic()

        # Publish odom -> base_link only when the driver does not already do it;
        # two publishers of the same transform make RViz report TF conflicts.
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_link"
            transform.transform.translation.x = pos.x
            transform.transform.translation.y = pos.y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.last_scan_time = time.monotonic()

    def on_goal_pose(self, msg: PoseStamped) -> None:
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.controller.reset()
        self.get_logger().info(f"new goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    # ------------------------------------------------------------------
    def _stop(self, reason: str) -> None:
        self.cmd_pub.publish(Twist())
        now = time.monotonic()
        if now - self._last_log > 2.0:
            self.get_logger().warn(f"stop: {reason}")
            self._last_log = now

    def _shift_window_if_needed(self) -> None:
        cx, cy = self._window_center
        if math.hypot(self.state.x - cx, self.state.y - cy) <= self.map_half * 0.5:
            return
        bounds = (self.state.x - self.map_half, self.state.y - self.map_half,
                  self.state.x + self.map_half, self.state.y + self.map_half)
        self.controller.shift_window(bounds)
        self._window_center = (self.state.x, self.state.y)
        self.get_logger().info("local map window shifted")

    def on_timer(self) -> None:
        now = time.monotonic()
        if self.last_odom_time is None or now - self.last_odom_time > self.data_timeout:
            self._stop("no fresh /odom")
            return
        if self.scan_msg is None or self.last_scan_time is None or \
                now - self.last_scan_time > self.data_timeout:
            self._stop("no fresh /scan")
            return

        if math.hypot(self.goal[0] - self.state.x, self.goal[1] - self.state.y) <= \
                self.controller.goal_tolerance:
            self._stop("goal reached")
            return

        self._shift_window_if_needed()

        points = scan_points_from_ranges(
            self.scan_msg.ranges, self.scan_msg.angle_min,
            self.scan_msg.angle_increment, self.state,
            self.scan_msg.range_min, self.scan_msg.range_max,
            max_points=self.max_scan_points,
            laser_x=self.laser_x, laser_y=self.laser_y)

        v, w = self.controller(self.state, self.goal, points)

        cmd = Twist()
        cmd.linear.x = float(max(-self.controller.params.max_v,
                                 min(self.controller.params.max_v, v)))
        cmd.angular.z = float(max(-self.controller.params.max_w,
                                  min(self.controller.params.max_w, w)))
        self.cmd_pub.publish(cmd)

        if now - self._last_log > 1.0:
            result = self.controller.last_result
            source = result.source if result is not None else "?"
            self.get_logger().info(
                f"pose=({self.state.x:.2f},{self.state.y:.2f},{self.state.yaw:.2f}) "
                f"cmd=({cmd.linear.x:.2f},{cmd.angular.z:.2f}) src={source} "
                f"target=({self.controller.target[0]:.2f},{self.controller.target[1]:.2f})")
            self._last_log = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DwaAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

