#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""牛耕式全覆盖规划 + 跟踪控制节点。

启动时用 ``coverage.plan`` 离线生成一次牛耕式全覆盖路径（L 形房间，
行距 ``gap``），把路径以 ``nav_msgs/Path`` 发布到 ``planning/path``；
随后持续订阅 ``odom``，反馈跟踪路径，以 ``geometry_msgs/Twist`` 发布到
``cmd_vel``——路径走完即视为覆盖完成，停车并打印预估作业时间。

与 potential_field_avoidance / bug2_navigation 保持同一模式：**离线规划
得到参考轨迹，控制端只做反馈跟踪**，数值可被 ``tests/test_coverage.py``
锁死。

完整闭环（配合 sim_node）：

    sim_node --/odom--> plan_control_node --/cmd_vel--> sim_node
    plan_control_node --/planning/path--> RViz / 下游

启动：

    ros2 launch boustrophedon_coverage boustrophedon_coverage.launch.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import coverage  # noqa: E402

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


def track_cmd(path, pose, idx, speed, arrive_r=0.55, w_k=2.5, w_max=2.0):
    """反馈跟踪（与 ROS 无关的纯函数，便于离线回放与单测）。

    从 ``idx`` 起把游标推进到路径上离当前位置最近（允许越过）的点，朝它
    开车；大航向误差自动减速。返回 ``(v, w, new_idx, done)``。
    """
    x, y, yaw = pose
    goal = path[-1]
    if math.hypot(x - goal[0], y - goal[1]) < arrive_r:
        return 0.0, 0.0, idx, True

    def d(p):
        return math.hypot(p[0] - x, p[1] - y)

    while idx + 1 < len(path) and d(path[idx + 1]) <= d(path[idx]):
        idx += 1
    tx, ty = path[min(idx + 1, len(path) - 1)]
    err = wrap(math.atan2(ty - y, tx - x) - yaw)
    v = speed * max(0.15, 1.0 - abs(err) / math.pi)
    w = max(-w_max, min(w_max, w_k * err))
    return v, w, idx, False


class CoverageControlNode(Node if HAS_ROS else object):
    """牛耕式全覆盖：离线规划一次，之后反馈跟踪直到路径走完。"""

    def __init__(self, v=2.0, rate=20.0, gap=2.0):
        super().__init__("coverage_planner_control")
        self.declare_parameter("v", float(v))
        self.declare_parameter("gap", float(gap))
        self.v = float(self.get_parameter("v").value)
        self.gap = float(self.get_parameter("gap").value)

        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "planning/path", 10)

        res = coverage.plan(coverage.ROOM_L, gap=self.gap)
        self.path = res["path"]
        self.est_time = res["est_time"]
        self.get_logger().info(
            "牛耕式覆盖路径: %d 行, 长度 %.1f m, 房间面积 %.0f m^2, "
            "预估作业 %.0f s (1 m/s)"
            % (res["rows"], res["length"], res["area"], res["est_time"])
        )

        self.pose = None
        self.idx = 0
        self.finished = False
        self.timer = self.create_timer(1.0 / float(rate), self._tick)
        self.get_logger().info("coverage_planner_control 就绪: 行距 %.1f m, 巡航 %.1f m/s"
                               % (self.gap, self.v))

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
        if not self.path:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        for (x, y) in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _tick(self):
        self._publish_path()
        if self.pose is None or self.finished:
            return

        cmd = Twist()
        v, w, self.idx, done = track_cmd(self.path, self.pose, self.idx,
                                         speed=self.v)
        if done:                                  # 路径走完 = 覆盖完成
            self.finished = True
            self.pub_cmd.publish(cmd)
            self.get_logger().info("覆盖完成（路径走完），停车")
            return
        if v == 0.0 and w == 0.0:
            return
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)


def strip_ros_args(argv):
    for i, a in enumerate(argv):
        if a == "--ros-args":
            return argv[:i]
    return argv


def parse_cli(argv=None):
    argv = strip_ros_args(list(sys.argv[1:] if argv is None else argv))
    ap = argparse.ArgumentParser(description="牛耕式全覆盖规划 + 跟踪控制节点")
    ap.add_argument("--v", type=float, default=2.0, help="巡航速度 (m/s)")
    ap.add_argument("--rate", type=float, default=20.0, help="控制频率 (Hz)")
    ap.add_argument("--gap", type=float, default=2.0, help="扫掠行距 (m)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = CoverageControlNode(v=args.v, rate=args.rate, gap=args.gap)
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
