#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""梯形 / S 曲线速度规划节点：把"走多远"变成"每时刻该多快"。

启动时沿 +x 方向生成一条长度 ``distance`` 的直线参考路径（``nav_msgs/Path``），
并按梯形或 S 曲线生成时间参数化曲线；运行中按节点时钟推进 ``t``，用
``profile.eval(t)`` 取出该时刻的期望速度，叠加横向/航向修正后以
``geometry_msgs/Twist`` 发到 ``cmd_vel``。

链路：

    sim_node --/odom--> plan_control_node --/cmd_vel--> sim_node
    plan_control_node --/planning/path--> RViz

启动：

    ros2 launch trapezoidal_profile trapezoidal_profile.launch.py
    ros2 launch trapezoidal_profile trapezoidal_profile.launch.py use_scurve:=true
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import profile as prof_mod  # noqa: E402

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


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ProfileControlNode(Node if HAS_ROS else object):
    """沿直线执行速度剖面，横向/航向做 PD 修正。"""

    def __init__(self, distance=10.0, vmax=1.5, amax=1.0, jmax=5.0,
                 use_scurve=False, rate=50.0, wheelbase=2.5, ky=1.2, kpsi=1.6):
        super().__init__("profile_control")
        self.declare_parameter("distance", float(distance))
        self.declare_parameter("vmax", float(vmax))
        self.declare_parameter("amax", float(amax))
        self.declare_parameter("jmax", float(jmax))
        self.declare_parameter("use_scurve", bool(use_scurve))
        self.declare_parameter("wheelbase", float(wheelbase))

        self.distance = float(self.get_parameter("distance").value)
        vmax = float(self.get_parameter("vmax").value)
        amax = float(self.get_parameter("amax").value)
        jmax = float(self.get_parameter("jmax").value)
        use_scurve = bool(self.get_parameter("use_scurve").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.ky, self.kpsi = float(ky), float(kpsi)

        if use_scurve:
            self.profile = prof_mod.SCurveProfile(self.distance, vmax, amax, jmax)
            self.kind = "S-CURVE"
        else:
            self.profile = prof_mod.TrapezoidProfile(self.distance, vmax, amax)
            self.kind = "TRAP"

        self.t = 0.0
        self.dt = 1.0 / float(rate)
        self.finished = False
        self.pose = None

        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "planning/path", 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            "%s 剖面就绪: 距离 %.2f m, vmax %.2f, amax %.2f, 总时长 %.3f s"
            % (self.kind, self.distance, vmax, amax, self.profile.T)
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
        n = 21
        for i in range(n):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = self.distance * i / (n - 1)
            ps.pose.position.y = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _tick(self):
        self._publish_path()
        if self.finished:
            return

        cmd = Twist()
        s_des, v_des, _a_des = self.profile.eval(self.t)
        self.t += self.dt

        # 横向与航向修正：把车压回 y=0、psi=0 这条直线
        if self.pose is not None:
            _x, y, psi = self.pose
            psi = wrap(psi)
            omega = -self.ky * y - self.kpsi * psi
        else:
            omega = 0.0
        omega = max(-1.0, min(1.0, omega))

        if self.t > self.profile.T + 0.5:      # 剖面结束 + 0.5 s 稳定余量
            self.finished = True
            self.pub_cmd.publish(cmd)
            self.get_logger().info(
                "剖面执行完毕: 目标 %.2f m，实际 x=%.3f m（误差 %.4f m）"
                % (self.distance,
                   self.pose[0] if self.pose else float("nan"),
                   abs(self.distance - (self.pose[0] if self.pose else 0.0)))
            )
            return

        cmd.linear.x = float(v_des)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)


def strip_ros_args(argv):
    for i, a in enumerate(argv):
        if a == "--ros-args":
            return argv[:i]
    return argv


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(description="梯形 / S 曲线速度规划节点")
    ap.add_argument("--distance", type=float, default=10.0, help="目标位移 (m)")
    ap.add_argument("--vmax", type=float, default=1.5, help="最大速度 (m/s)")
    ap.add_argument("--amax", type=float, default=1.0, help="最大加速度 (m/s^2)")
    ap.add_argument("--jmax", type=float, default=5.0, help="最大 jerk (m/s^3)，仅 S 曲线用")
    ap.add_argument("--use-scurve", action="store_true", help="使用 S 曲线（默认梯形）")
    ap.add_argument("--rate", type=float, default=50.0, help="控制频率 (Hz)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = ProfileControlNode(
        distance=args.distance, vmax=args.vmax, amax=args.amax, jmax=args.jmax,
        use_scurve=args.use_scurve, rate=args.rate,
    )
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
