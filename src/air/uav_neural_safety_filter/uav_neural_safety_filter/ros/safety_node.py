#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""神经安全过滤节点（ROS2 Humble，Python 3.8）。

订阅 `/lidar/points`、`/uav/odom` 与标称指令 `/uav/cmd_vel_nominal`，
以 20 Hz 输出修正后的 `/uav/cmd_vel`。

坐标系
------
训练时的动作是**机头系**速度；激光点云来自底座，是"相对机体中心、世界轴向"
的坐标，节点先旋到机头系再做距离栅格：

    p_heading = R_z(yaw)^T · p_rel

发布前再把修正指令转回机体系：

    v_body = R^T · R_z(yaw) · u

这样底座的全姿态映射 `R · v_body` 复原出的世界速度正好是 `R_z(yaw)·u`。
"""
from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from ..core.filter import FilterConfig, LearnedFilter
from ..core.network import PolicyNet
from ..core.ranges import MAX_RANGE, build_features, points_to_grid

_SENSOR_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def cloud_to_points(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N,3) float32；字段按 3×float32 解释。"""
    step = int(msg.point_step) // 4
    a = np.frombuffer(msg.data, np.float32)
    if step <= 0 or a.size % step:
        return np.zeros((0, 3), np.float32)
    return a.reshape(-1, step)[:, :3].astype(np.float32)


class SafetyNode(Node):
    def __init__(self):
        super().__init__("uav_safety_filter")
        p = self.declare_parameter
        p("model", "")
        p("topic.lidar", "/lidar/points")
        p("topic.odom", "/uav/odom")
        p("topic.cmd_in", "/uav/cmd_vel_nominal")
        p("topic.cmd_out", "/uav/cmd_vel")
        p("topic.goal", "/uav/goal")
        p("topic.status", "/safety/status")
        p("rate_hz", 20.0)
        p("max_horiz", 4.0)
        p("max_vert", 2.0)
        p("guard_m", 0.0)
        p("dir_guard", 0.0)

        g = lambda k: self.get_parameter(k).value            # noqa: E731
        self.cfg = FilterConfig(max_horiz=float(g("max_horiz")),
                                max_vert=float(g("max_vert")),
                                guard_m=float(g("guard_m")),
                                dir_guard=float(g("dir_guard")))
        self.net = self._load(str(g("model")))
        self.filt = LearnedFilter(self.net, self.cfg) if self.net else None

        self.ranges = None
        self.lidar_t = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.goal = None
        self.cmd_in = np.zeros(3)
        self.cmd_t = 0.0
        self.n_infer = 0
        self.n_guard = 0
        self.lat = 0.0
        self.last_cmd = np.zeros(3)
        self.t0 = time.time()

        self.create_subscription(PointCloud2, str(g("topic.lidar")), self.on_lidar,
                                 _SENSOR_QOS)
        self.create_subscription(Odometry, str(g("topic.odom")), self.on_odom, 10)
        self.create_subscription(Twist, str(g("topic.cmd_in")), self.on_cmd, 10)
        self.create_subscription(Point, str(g("topic.goal")), self.on_goal, 10)
        self.pub_cmd = self.create_publisher(Twist, str(g("topic.cmd_out")), 10)
        self.pub_status = self.create_publisher(String, str(g("topic.status")), 10)
        self.create_timer(1.0 / max(float(g("rate_hz")), 1e-3), self.tick)
        self.create_timer(5.0, self.report)
        if self.net is None:
            self.get_logger().error("安全层权重未加载，节点不会发指令")
        else:
            self.get_logger().info("安全过滤节点启动：%s" % self.net.summary())

    def _load(self, path):
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory(
                    "uav_neural_safety_filter"), "data", "safety_policy.npz")
            except Exception:                                    # noqa: BLE001
                path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                    "..", "data", "safety_policy.npz")
        try:
            return PolicyNet.load(path)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn("加载 %s 失败：%s" % (path, e))
            return None

    # ------------------------------------------------------------------ 回调
    def on_lidar(self, m: PointCloud2) -> None:
        pts = cloud_to_points(m)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        pts_h = pts @ Rz
        self.ranges = points_to_grid(pts_h, max_range=MAX_RANGE)
        self.lidar_t = time.time()

    def on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self.quat = np.array([q.w, q.x, q.y, q.z])
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])
        v = m.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_cmd(self, m: Twist) -> None:
        self.cmd_in = np.array([m.linear.x, m.linear.y, m.linear.z])
        self.cmd_t = time.time()

    def on_goal(self, m: Point) -> None:
        self.goal = np.array([m.x, m.y, m.z])

    # ------------------------------------------------------------------ 主循环
    def tick(self) -> None:
        if self.filt is None:
            return
        now = time.time()
        if (self.ranges is None or now - self.lidar_t > 1.0
                or now - self.cmd_t > 0.5):
            self._publish(np.zeros(3))
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        vel_h = Rz.T @ self.vel
        goal = self.goal if self.goal is not None else self.pos + Rz[:, 0] * 10.0
        dp = goal - self.pos
        dist = float(np.linalg.norm(dp))
        goal_dir = Rz.T @ (dp / max(dist, 1e-6))
        feat = build_features(self.ranges, vel_h, goal_dir, dist,
                              float(self.pos[2]))
        obs = {"ranges": self.ranges, "feat": feat.astype(np.float32),
               "vel": vel_h, "goal_dir": goal_dir, "dist": dist,
               "pos": self.pos, "yaw": self.yaw, "h": 0.0}
        t0 = time.perf_counter()
        u, _ = self.filt.act(obs, self.cmd_in)
        self.lat = (time.perf_counter() - t0) * 1e3
        self.n_infer += 1
        if not np.any(np.abs(u) > 1e-9) and np.any(np.abs(self.cmd_in) > 1e-9):
            self.n_guard += 1
        self._publish(u)

    def _publish(self, a: np.ndarray) -> None:
        self.last_cmd = np.asarray(a, float)
        R = quat_to_mat(self.quat)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        world = np.array([c * a[0] - s * a[1], s * a[0] + c * a[1], a[2]])
        v_body = R.T @ world
        t = Twist()
        t.linear.x = float(v_body[0])
        t.linear.y = float(v_body[1])
        t.linear.z = float(v_body[2])
        self.pub_cmd.publish(t)

    def report(self) -> None:
        now = time.time()
        s = String()
        s.data = ("infer=%d guard=%d freq=%.1fHz latency=%.2fms "
                  "cmd=(%.2f,%.2f,%.2f) pos=(%.1f,%.1f,%.1f)"
                  % (self.n_infer, self.n_guard,
                     self.n_infer / max(now - self.t0, 1e-6), self.lat,
                     self.last_cmd[0], self.last_cmd[1], self.last_cmd[2],
                     self.pos[0], self.pos[1], self.pos[2]))
        self.pub_status.publish(s)
        self.get_logger().info(s.data)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
