#!/usr/bin/env python3
"""ROS 2 node for the neural collision-avoidance stack.

The same node serves every module; launch files select it with the ``module``
parameter (``perception``, ``planning``, ``control``, ``e2e`` or ``full``).
"""

from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import add_repo_to_path, decode_depth, quat_to_yaw

_ROOT = add_repo_to_path()

from ca_drone.control.expert import lookahead_point  # noqa: E402
from ca_drone.mapping import GridMap, patch_clearance  # noqa: E402
from ca_drone.models.control import control_features, predict_command  # noqa: E402
from ca_drone.models.e2e import goal_features  # noqa: E402
from ca_drone.models.perception import predict_mask  # noqa: E402
from ca_drone.nn.autograd import Tensor  # noqa: E402
from ca_drone.pipeline import (DEPTH_H, DEPTH_W, FOV, MAX_CMD, MAX_RANGE,  # noqa: E402
                               NeuralStack, _plan)


class NeuralDroneNode(Node):
    def __init__(self):
        super().__init__("neural_ca_drone")
        self.declare_parameter("module", "full")
        self.declare_parameter("weights", os.path.join(_ROOT, "weights"))
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("goal_x", 49.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 3.0)
        self.declare_parameter("replan_period", 0.5)

        self.module = self.get_parameter("module").value
        self.weights = self.get_parameter("weights").value
        rate = float(self.get_parameter("rate").value)
        self.goal = np.array([
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value)])
        self.replan_period = float(self.get_parameter("replan_period").value)

        self.stack = NeuralStack(self.weights)
        self.gm = GridMap(res=0.5, x0=0.0, y0=-22.0, nx=104, ny=88)
        self.depth = None
        self.depth_time = 0.0
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.vel = np.zeros(3)
        self.path = None
        self.last_plan = -1e9
        self.last_plan_time = 0.0
        self.frames = 0

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_occ = self.create_publisher(OccupancyGrid, "/ca/occupancy", 1)
        self.pub_path = self.create_publisher(Path, "/ca/path", 1)
        self.pub_status = self.create_publisher(String, "/ca/status", 1)

        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_cb, 1)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.goal_cb, 1)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("neural_ca_drone started: module=%s weights=%s" % (
            self.module, self.weights))

    def depth_cb(self, msg: Image):
        try:
            self.depth = decode_depth(msg, DEPTH_W, DEPTH_H, MAX_RANGE)
            self.depth_time = time.monotonic()
        except Exception as exc:  # pragma: no cover - driver dependent
            self.get_logger().warn("depth decode failed: %s" % exc)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def goal_cb(self, msg: PoseStamped):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y,
                              msg.pose.position.z or 3.0])
        self.get_logger().info("new goal: %s" % np.round(self.goal, 2))

    def tick(self):
        if self.depth is None:
            self.pub_status.publish(String(data="waiting for /camera/depth/image_raw"))
            return
        if time.monotonic() - self.depth_time > 1.0:
            self.pub_cmd.publish(Twist())
            self.pub_status.publish(String(data="stale depth -> hover"))
            return

        depth = self.depth
        if self.module in ("perception", "planning", "control", "full"):
            prob = predict_mask(self.stack.perception, depth, MAX_RANGE)
            self.gm.integrate_depth(depth, prob, self.pos, self.yaw, FOV, MAX_RANGE,
                                    threshold=0.55)
        if self.module in ("perception", "full"):
            self.publish_occupancy()
        if self.module in ("planning", "control", "full"):
            self.maybe_plan()
        if self.module in ("control", "full"):
            self.publish_command(self.control_command())
        elif self.module == "e2e":
            self.publish_command(self.e2e_command(depth))
        elif self.module == "planning":
            self.pub_cmd.publish(Twist())

    def maybe_plan(self):
        now = time.monotonic()
        if self.path is not None and now - self.last_plan_time < self.replan_period:
            return
        r = _plan(self.stack, self.gm, self.pos, self.goal, use_neural=True)
        if r.get("success"):
            self.path = r["world_path"]
            self.publish_path(self.path)
        self.last_plan_time = now

    def control_command(self) -> np.ndarray:
        if self.path is None or len(self.path) < 2:
            return np.zeros(2)
        target = lookahead_point(self.path, self.pos, 2.0)
        delta = target[:2] - self.pos[:2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
        patch = self.gm.local_patch(self.pos, 9)
        feats = control_features(patch, gb, np.array([self.vel[0], self.vel[1]]),
                                 patch_clearance(patch, 0.5))
        cmd = predict_command(self.stack.control, feats)
        n = float(np.linalg.norm(cmd))
        return cmd * (MAX_CMD / n) if n > MAX_CMD else cmd

    def e2e_command(self, depth: np.ndarray) -> np.ndarray:
        delta = self.goal[:2] - self.pos[:2]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        gb = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])
        feat = goal_features(gb, float(np.linalg.norm(delta)))
        out = self.stack.e2e(Tensor(np.clip(depth / MAX_RANGE, 0, 1)[None, None]),
                             Tensor(feat[None]))
        return np.asarray(out.data[0], dtype=float)

    def publish_command(self, cmd: np.ndarray):
        msg = Twist()
        msg.linear.x = float(np.clip(cmd[0], -1.5, 1.5))
        msg.linear.y = float(np.clip(cmd[1], -1.5, 1.5))
        speed = float(np.hypot(msg.linear.x, msg.linear.y))
        if speed > 0.15:
            cmd_yaw = math.atan2(msg.linear.y, msg.linear.x)
            err = (cmd_yaw + self.yaw + math.pi) % (2 * math.pi) - math.pi
            msg.angular.z = float(np.clip(1.5 * err, -1.2, 1.2))
        self.pub_cmd.publish(msg)
        self.pub_status.publish(String(data=(
            "module=%s cmd=(%.2f,%.2f) yaw_rate=%.2f speed=%.2f" % (
                self.module, msg.linear.x, msg.linear.y, msg.angular.z, speed))))

    def publish_occupancy(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.5
        grid.info.width = self.gm.nx
        grid.info.height = self.gm.ny
        grid.info.origin.position.x = self.gm.x0
        grid.info.origin.position.y = self.gm.y0
        grid.data = (self.gm.occ.astype(np.int8).reshape(-1) * 100).tolist()
        self.pub_occ.publish(grid)

    def publish_path(self, pts: np.ndarray):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for p in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2]) if len(p) > 2 else 3.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NeuralDroneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
