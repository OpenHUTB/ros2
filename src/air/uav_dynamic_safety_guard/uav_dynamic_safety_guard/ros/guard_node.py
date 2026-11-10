#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 在线节点：三帧距离栅格 + 标称指令 → 修正指令。

话题约定与 ``uav_mujoco_sim`` / ``carlair_ros_bridge`` 一致：
``/lidar/points`` 是相对机体中心、世界轴向的点云，``/uav/cmd_vel`` 是机体系
速度指令。历史帧在节点内维护，因此网络看到的时序特征与离线训练完全一致。
"""
from __future__ import annotations

import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from ..core.network import GuardNet
from ..core.ranges import (HISTORY, MAX_RANGE, build_sequence_features,
                           points_to_grid)

__all__ = ["GuardNode", "cloud_to_points", "quat_to_yaw", "main"]


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cloud_to_points(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N,3) float32；按字段偏移解析，兼容带填充的点结构。"""
    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 3), np.float32)
    off = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            off[f.name] = int(f.offset)
    if len(off) != 3:
        return np.zeros((0, 3), np.float32)
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]],
                   "itemsize": int(msg.point_step)})
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)


class GuardNode(Node):
    """订阅点云/里程计/标称指令，发布修正指令与状态。"""

    def __init__(self) -> None:
        super().__init__("uav_dynamic_safety_guard")
        p = self.declare_parameter
        self.topic_lidar = str(p("topic.lidar", "/lidar/points").value)
        self.topic_odom = str(p("topic.odom", "/uav/odom").value)
        self.topic_cmd_in = str(p("topic.cmd_in", "/uav/cmd_vel_nominal").value)
        self.topic_cmd_out = str(p("topic.cmd_out", "/uav/cmd_vel").value)
        self.topic_goal = str(p("topic.goal", "/uav/goal").value)
        self.topic_status = str(p("topic.status", "/guard/status").value)
        self.rate_hz = float(p("rate_hz", 20.0).value)
        self.max_horiz = float(p("max_horiz", 4.0).value)
        self.max_vert = float(p("max_vert", 2.0).value)
        self.guard_m = float(p("guard_m", 0.0).value)
        self.dir_guard = float(p("dir_guard", 0.0).value)
        self.stale_s = float(p("stale_s", 0.5).value)
        self.model_path = str(p("model", "").value)

        self.net = self._load(self.model_path)
        self.grids = deque(maxlen=HISTORY)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.goal = np.zeros(3)
        self.u_nom = np.zeros(3)
        self.t_lidar = 0.0
        self.t_odom = 0.0
        self.t_cmd = 0.0
        self.n_infer = 0
        self.lat_ms = 0.0

        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, self.topic_lidar,
                                 self.on_lidar, qos)
        self.create_subscription(Odometry, self.topic_odom, self.on_odom, 10)
        self.create_subscription(Twist, self.topic_cmd_in, self.on_cmd, 10)
        self.create_subscription(Point, self.topic_goal, self.on_goal, 10)
        self.pub_cmd = self.create_publisher(Twist, self.topic_cmd_out, 10)
        self.pub_status = self.create_publisher(String, self.topic_status, 10)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1e-3), self.tick)
        self.get_logger().info(
            "动态安全层就绪：%s → %s（模型 %s）"
            % (self.topic_cmd_in, self.topic_cmd_out, self.model_path or "内置"))

    # ------------------------------------------------------------------ 载入
    def _load(self, path: str) -> GuardNet:
        candidates = []
        if path:
            candidates.append(path)
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("uav_dynamic_safety_guard")
            candidates.append(os.path.join(share, "data",
                                           "dynamic_policy_seq.npz"))
        except Exception:                                # noqa: BLE001
            pass
        here = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        candidates.append(os.path.join(here, "data", "dynamic_policy_seq.npz"))
        for c in candidates:
            if c and os.path.exists(c):
                self.get_logger().info("载入模型 %s" % c)
                return GuardNet.load(c)
        raise RuntimeError("找不到安全层模型：%s" % candidates)

    # ------------------------------------------------------------------ 回调
    def on_lidar(self, m: PointCloud2) -> None:
        pts = cloud_to_points(m)
        if len(pts) == 0:
            return
        if len(self.grids) == 0:                 # 冷启动：先塞满历史帧
            grid = points_to_grid(self._to_body(pts))
            for _ in range(HISTORY):
                self.grids.append(grid)
        else:
            self.grids.append(points_to_grid(self._to_body(pts)))
        self.t_lidar = time.time()

    def _to_body(self, pts: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        return pts @ rot.T

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        v = m.twist.twist.linear
        q = m.pose.pose.orientation
        self.pos = np.array([p.x, p.y, p.z], float)
        self.vel = np.array([v.x, v.y, v.z], float)
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.t_odom = time.time()

    def on_cmd(self, m: Twist) -> None:
        self.u_nom = np.array([m.linear.x, m.linear.y, m.linear.z], float)
        self.t_cmd = time.time()

    def on_goal(self, m: Point) -> None:
        self.goal = np.array([m.x, m.y, m.z], float)

    # ------------------------------------------------------------------ 主循环
    def tick(self) -> None:
        now = time.time()
        if (len(self.grids) == 0 or now - self.t_lidar > self.stale_s
                or now - self.t_odom > self.stale_s):
            self._publish(np.zeros(3))
            return
        hist = np.asarray(self.grids, np.float32)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        vel_b = rot.T @ self.vel
        dp = rot.T @ (self.goal - self.pos)
        dist = float(np.linalg.norm(dp))
        goal_dir = dp / max(dist, 1e-6)
        feat = build_sequence_features(hist, vel_b, goal_dir, dist,
                                       float(self.pos[2]))
        t0 = time.perf_counter()
        x = np.concatenate([feat, self.u_nom])[None, :]
        u = np.asarray(self.net.forward(x)[0], float)
        self.lat_ms = (time.perf_counter() - t0) * 1e3
        u = np.array([np.clip(u[0], -1.5, self.max_horiz),
                      np.clip(u[1], -3.0, 3.0),
                      np.clip(u[2], -self.max_vert, self.max_vert)])
        if self.guard_m > 0 and float(np.min(hist[0])) < self.guard_m:
            u = np.zeros(3)
        if self.dir_guard > 0 and math.hypot(u[0], u[1]) > 0.2:
            az = math.atan2(u[1], u[0])
            ai = int(round((az + math.pi) / (2 * math.pi)
                           * hist.shape[2])) % hist.shape[2]
            r = float(hist[:, :, ai].min())
            if r < self.dir_guard:
                u = u * max(0.0, (r - 0.3)
                            / max(self.dir_guard - 0.3, 1e-6))
        self.n_infer += 1
        self._publish(u)
        if self.n_infer % 20 == 0:
            self.pub_status.publish(String(data=json.dumps(
                {"n": self.n_infer, "latency_ms": round(self.lat_ms, 3),
                 "action": [round(float(v), 3) for v in u],
                 "min_range": round(float(np.min(hist[0])), 2),
                 "h_margin": round(self.guard_m, 2)}, ensure_ascii=False)))

    def _publish(self, u: np.ndarray) -> None:
        msg = Twist()
        msg.linear.x = float(u[0])
        msg.linear.y = float(u[1])
        msg.linear.z = float(u[2])
        self.pub_cmd.publish(msg)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = GuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:                            # pragma: no cover
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:                                # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":                               # pragma: no cover
    raise SystemExit(main())
