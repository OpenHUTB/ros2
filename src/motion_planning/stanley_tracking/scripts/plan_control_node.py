#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stanley 前视路径跟踪节点（含曲率前馈）。

订阅 ``odom`` 拿到车辆位姿，对参考圆路径求最近点与横向偏差 ``e``、航向偏差
``psi_e``，按 Stanley 控制律

    delta = psi_e + arctan(k e / v) + arctan(L * kappa)

算出前轮转角，再转成角速度以 ``geometry_msgs/Twist`` 发到 ``cmd_vel``；
同时把参考路径以 ``nav_msgs/Path`` 发到 ``planning/path`` 便于 RViz 观察。

链路：

    sim_node --/odom--> stanley_control --/cmd_vel--> sim_node
    stanley_control --/planning/path--> RViz

启动：

    ros2 launch stanley_tracking stanley_tracking.launch.py
    # 关掉曲率前馈，观察弯道上的稳态偏差
    ros2 launch stanley_tracking stanley_tracking.launch.py feedforward:=false
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import stanley as st  # noqa: E402

try:  # ROS 2 为可选依赖
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path

    HAS_ROS = True
except ImportError:  # pragma: no cover - 无 ROS 环境
    rclpy = None
    Node = object
    PoseStamped = None
    Twist = None
    Odometry = None
    Path = None
    HAS_ROS = False


class StanleyControlNode(Node if HAS_ROS else object):
    """跟踪半径可配的圆路径。"""

    def __init__(self, radius=8.0, v=2.0, k=1.2, feedforward=True,
                 rate=50.0, wheelbase=None):
        super().__init__("stanley_control")
        default_l = float(wheelbase if wheelbase is not None else st.L_WHEELBASE)
        self.declare_parameter("radius", float(radius))
        self.declare_parameter("v", float(v))
        self.declare_parameter("k", float(k))
        self.declare_parameter("feedforward", bool(feedforward))
        self.declare_parameter("wheelbase", default_l)

        self.path = st.CirclePath(float(self.get_parameter("radius").value))
        self.v = float(self.get_parameter("v").value)
        self.k = float(self.get_parameter("k").value)
        self.feedforward = bool(self.get_parameter("feedforward").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.dt = 1.0 / float(rate)

        self.pose = None
        self.last_e = 0.0
        self.last_delta = 0.0

        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "planning/path", 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            "stanley_control 就绪: R=%.2f m, v=%.2f m/s, k=%.2f, 前馈=%s"
            % (radius, self.v, self.k, self.feedforward)
        )

    # ---------- 回调 ----------
    def _on_odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (float(p.position.x), float(p.position.y), yaw)

    def _publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        n = 181
        circumference = 2.0 * math.pi * self.path.R
        for i in range(n):
            x, y = self.path.point_at(circumference * i / (n - 1))
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _tick(self):
        self._publish_path()
        if self.pose is None:
            return

        x, y, psi = self.pose
        _pt, psi_ref, e = self.path.nearest(x, y)
        psi_e = st.wrap(psi_ref - psi)
        kappa = self.path.curvature(x, y)

        delta = st.stanley_steer(e, psi_e, self.v, self.k,
                                 curvature=kappa, L=self.wheelbase,
                                 feedforward=self.feedforward)
        self.last_e, self.last_delta = e, delta

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.v / self.wheelbase * math.tan(delta)
        self.pub_cmd.publish(cmd)


def strip_ros_args(argv):
    for i, a in enumerate(argv):
        if a == "--ros-args":
            return argv[:i]
    return argv


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(description="Stanley 前视路径跟踪节点")
    ap.add_argument("--radius", type=float, default=8.0, help="参考圆半径 (m)")
    ap.add_argument("--v", type=float, default=2.0, help="巡航速度 (m/s)")
    ap.add_argument("--k", type=float, default=1.2, help="横向增益")
    ap.add_argument("--no-feedforward", action="store_true", help="关闭曲率前馈（对照实验）")
    ap.add_argument("--rate", type=float, default=50.0, help="控制频率 (Hz)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = StanleyControlNode(radius=args.radius, v=args.v, k=args.k,
                              feedforward=not args.no_feedforward, rate=args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
