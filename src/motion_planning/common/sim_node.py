#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动学自行车模型模拟器：订阅 ``cmd_vel``，发布 ``odom``。

这是 ROS 图里"仿真器 + 传感器"的那一侧：把控制指令积分成位姿，再以里程计
形式对外发布，供规划/控制节点订阅。与 ``plan_control_node`` 组成最小闭环：

    plan_control_node --/cmd_vel--> sim_node --/odom--> plan_control_node

也可以只起这一个节点，用别的节点或 ``ros2 topic pub`` 发 ``cmd_vel`` 单独验证链路。
"""

from __future__ import annotations

import argparse
import math
import sys

try:  # ROS 2 为可选依赖
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    HAS_ROS = True
except ImportError:  # pragma: no cover - 无 ROS 环境
    rclpy = None
    Node = object
    Twist = None
    Odometry = None
    HAS_ROS = False


def wrap(a):
    """把角度归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class KinematicSimNode(Node if HAS_ROS else object):
    """二维自行车模型：x_dot = v cos(psi)，y_dot = v sin(psi)，psi_dot = omega。"""

    def __init__(self, rate=50.0, x0=0.0, y0=0.0, yaw0=0.0,
                 v_max=4.0, w_max=2.0):
        super().__init__("kinematic_sim")
        # 声明参数：命令行默认值作为初值，launch / --ros-args 可覆盖
        self.declare_parameter("x0", float(x0))
        self.declare_parameter("y0", float(y0))
        self.declare_parameter("yaw0", float(yaw0))
        self.declare_parameter("v_max", float(v_max))
        self.declare_parameter("w_max", float(w_max))

        self.rate = float(rate)
        self.dt = 1.0 / self.rate
        self.x = float(self.get_parameter("x0").value)
        self.y = float(self.get_parameter("y0").value)
        self.yaw = float(self.get_parameter("yaw0").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)
        self.v, self.w = 0.0, 0.0

        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            "kinematic_sim 就绪: rate=%.0fHz 起点=(%.2f, %.2f, %.2frad)"
            % (self.rate, self.x, self.y, self.yaw)
        )

    def _on_cmd(self, msg):
        self.v = max(-self.v_max, min(self.v_max, float(msg.linear.x)))
        self.w = max(-self.w_max, min(self.w_max, float(msg.angular.z)))

    def _tick(self):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw = wrap(self.yaw + self.w * self.dt)

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.pub_odom.publish(od)


def strip_ros_args(argv):
    """把 ``--ros-args`` 及其后内容留给 rclpy。"""
    for i, a in enumerate(argv):
        if a == "--ros-args":
            return argv[:i]
    return argv


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(description="二维运动学模拟器节点（仿真器一侧）")
    ap.add_argument("--rate", type=float, default=50.0, help="积分频率 (Hz)")
    ap.add_argument("--x0", type=float, default=0.0)
    ap.add_argument("--y0", type=float, default=0.0)
    ap.add_argument("--yaw0", type=float, default=0.0)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = KinematicSimNode(rate=args.rate, x0=args.x0, y0=args.y0, yaw0=args.yaw0)
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
