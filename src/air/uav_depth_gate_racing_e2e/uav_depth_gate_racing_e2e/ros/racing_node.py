#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""穿越门竞速策略节点（ROS2 Humble，Python 3.8）。

订阅 `/camera/depth`（32FC1，米）与 `/uav/odom`，以 20 Hz 跑训练好的策略网络，
输出速度指令到 `/uav/cmd_vel`。

坐标系
------
策略的动作语义是**航向系**速度（与训练时 ``GateRacingEnv`` 完全一致），而
底座的 ``VelocityCommander`` 按**实时全姿态**把机体系指令旋到世界系，两者
不等价：机身为了前进俯仰 20°，前向指令就有约三分之一被旋到竖直轴上。节点
在发布前先把倾斜旋出去

    v_body = Rᵀ · R_z(yaw) · a

底座再左乘 R，复原出的世界速度正好是 ``R_z(yaw)·a``。

发布
----
* ``/uav/cmd_vel``       geometry_msgs/Twist，机体系
* ``/racing/status``     std_msgs/String
* ``/racing/depth_view`` sensor_msgs/Image（mono8），网络实际看到的深度图
"""
from __future__ import annotations

import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..core.gate_env import DEPTH_MAX, DEPTH_MIN, depth_to_u8
from ..core.policy import Policy

_SENSOR_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)


def quat_to_mat(q) -> np.ndarray:
    """四元数 (w,x,y,z) → 3×3 旋转矩阵（机体→世界）。"""
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def depth_to_net(d: np.ndarray, hw) -> np.ndarray:
    """深度（米）缩放到网络输入尺寸并量化成 u8。

    先按米平均再量化，不是先量化再缩放：量化是 1/254 的定点，先量化会把远处
    （一格十几厘米）的细节直接抹平。无效像素不参与平均，否则空洞会把周围的
    有效深度拉浅——按有效像素数加权。放大时退回最近邻，分箱会取到空箱。
    """
    h, w = d.shape[:2]
    oh, ow = hw
    if (h, w) == (oh, ow):
        return depth_to_u8(np.asarray(d, np.float32))
    ih = (np.arange(oh) * h) // oh
    iw = (np.arange(ow) * w) // ow
    if oh >= h or ow >= w:
        return depth_to_u8(np.asarray(d[np.ix_(ih, iw)], np.float32))
    valid = np.isfinite(d) & (d > DEPTH_MIN) & (d < DEPTH_MAX)
    dv = np.where(valid, d, 0.0)
    s = np.add.reduceat(np.add.reduceat(dv, ih, axis=0), iw, axis=1)
    n = np.add.reduceat(np.add.reduceat(valid.astype(np.float64), ih, axis=0),
                        iw, axis=1)
    return depth_to_u8(np.where(n > 0.0, s / np.maximum(n, 1.0), 0.0))


class RacingNode(Node):
    def __init__(self):
        super().__init__("uav_gate_racing")
        p = self.declare_parameter
        p("model", "")
        p("topic.depth", "/camera/depth")
        p("topic.odom", "/uav/odom")
        p("topic.cmd", "/uav/cmd_vel")
        p("topic.status", "/racing/status")
        p("topic.view", "/racing/depth_view")
        p("rate_hz", 20.0)
        p("publish_view", True)
        p("max_horiz", 4.0)          # 与 EnvConfig 对齐
        p("max_vert", 2.0)

        g = lambda k: self.get_parameter(k).value            # noqa: E731
        self.max_h = float(g("max_horiz"))
        self.max_v = float(g("max_vert"))
        self.pub_view = bool(g("publish_view"))
        self.model = self._load(str(g("model")))

        self.depth = None
        self.depth_t = 0.0
        self.pos = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.yaw = 0.0
        self.vel = np.zeros(3)
        self.h = self.model.init_hidden(1) if self.model else None
        self.n_infer = 0
        self.lat_ms = 0.0
        self.last_cmd = np.zeros(3)
        self.t0 = time.time()
        self.last_report = time.time()

        self.create_subscription(Image, str(g("topic.depth")), self.on_depth,
                                 _SENSOR_QOS)
        self.create_subscription(Odometry, str(g("topic.odom")), self.on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, str(g("topic.cmd")), 10)
        self.pub_status = self.create_publisher(String, str(g("topic.status")), 10)
        self.pub_img = (self.create_publisher(Image, str(g("topic.view")), 2)
                        if self.pub_view else None)
        self.create_timer(1.0 / max(float(g("rate_hz")), 1e-3), self.tick)
        self.create_timer(5.0, self.report)
        if self.model is None:
            self.get_logger().error("策略权重未加载，节点不会发指令")
        else:
            self.get_logger().info("竞速节点启动：%s" % self.model.summary())

    def _load(self, path):
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory(
                    "uav_depth_gate_racing_e2e"), "data", "policy_cnn.npz")
            except Exception:                                    # noqa: BLE001
                path = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                    "data", "policy_cnn.npz")
        try:
            return Policy.load(path)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn("加载 %s 失败：%s" % (path, e))
            return None

    # ------------------------------------------------------------------ 回调
    def on_depth(self, m: Image) -> None:
        if m.encoding != "32FC1":
            self.get_logger().warn("深度图编码为 %s，期望 32FC1" % m.encoding)
            return
        a = np.frombuffer(m.data, np.float32)
        if a.size != m.height * m.width:
            return
        self.depth = a.reshape(m.height, m.width).astype(np.float32)
        self.depth_t = time.time()

    def on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        self.quat = np.array([q.w, q.x, q.y, q.z])
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])
        v = m.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ------------------------------------------------------------------ 主循环
    def tick(self) -> None:
        if self.model is None:
            return
        if self.depth is None or time.time() - self.depth_t > 1.0:
            self._publish(np.zeros(3))
            return
        img = depth_to_net(self.depth, self.model.cfg.depth_hw)
        # 速度观测在机体系——与训练时 GateRacingEnv.observe 的 vel 同一语义
        obs = {"depth": img, "vel": quat_to_mat(self.quat).T @ self.vel}
        t0 = time.perf_counter()
        act, _, _, self.h, _ = self.model.act(obs, self.h, deterministic=True)
        self.lat_ms = (time.perf_counter() - t0) * 1e3
        self.n_infer += 1
        a = np.asarray(act[0], float)
        self._publish(np.array([np.clip(a[0], -self.max_h, self.max_h),
                                np.clip(a[1], -self.max_h, self.max_h),
                                np.clip(a[2], -self.max_v, self.max_v)]))
        if self.pub_img is not None:
            self._publish_view(img)

    def _publish(self, a: np.ndarray) -> None:
        self.last_cmd = a
        R = quat_to_mat(self.quat)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        world = np.array([c * a[0] - s * a[1], s * a[0] + c * a[1], a[2]])
        v_body = R.T @ world                       # 抵消底座的全姿态映射
        t = Twist()
        t.linear.x = float(v_body[0])
        t.linear.y = float(v_body[1])
        t.linear.z = float(v_body[2])
        self.pub_cmd.publish(t)

    def _publish_view(self, img: np.ndarray) -> None:
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "camera"
        m.height, m.width = int(img.shape[0]), int(img.shape[1])
        m.encoding = "mono8"
        m.is_bigendian = 0
        m.step = int(img.shape[1])
        m.data = img.tobytes()
        self.pub_img.publish(m)

    def report(self) -> None:
        now = time.time()
        self.last_report = now
        s = String()
        s.data = ("infer=%d freq=%.1fHz latency=%.2fms cmd=(%.2f,%.2f,%.2f) "
                  "pos=(%.1f,%.1f,%.1f) yaw=%.0f度"
                  % (self.n_infer, self.n_infer / max(now - self.t0, 1e-6),
                     self.lat_ms, self.last_cmd[0], self.last_cmd[1],
                     self.last_cmd[2], self.pos[0], self.pos[1], self.pos[2],
                     math.degrees(self.yaw)))
        self.pub_status.publish(s)
        self.get_logger().info(s.data)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = RacingNode()
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
