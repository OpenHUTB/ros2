#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RRT 规划 + 纯跟踪控制节点。

启动时用 RRT 在二维地图上规划一条无碰撞路径（再做随机捷径平滑），以
``nav_msgs/Path`` 发布到 ``planning/path``（RViz 可直接可视化）；随后持续订阅
``odom``，用纯跟踪（pure pursuit）算出前轮转角，以 ``geometry_msgs/Twist``
发布到 ``cmd_vel``。

平滑只留下约 10 个顶点、每段可长达数十米。直接把「下一个顶点」当前视目标会
让车辆朝拐角内侧切过去（实测 13.2 s 撞进第一个障碍）。因此这里做两件事：
先用 ``rrt.densify(..., 0.5)`` 把路径加密到 <=0.5 m，再用
``rrt.lookahead_point`` 按弧长插值求精确前视点。``tests/test_rrt.py`` 里有
「跟踪轨迹与障碍无交」的回归测试锁死这个缺陷。

完整闭环（配合 sim_node）：

    sim_node --/odom--> plan_control_node --/cmd_vel--> sim_node
    plan_control_node --/planning/path--> RViz / 下游

启动：

    ros2 launch rrt_path_planning rrt_path_planning.launch.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rrt  # noqa: E402

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


def pure_pursuit_cmd(path, pose, lookahead=8.0, v_cruise=3.0,
                     wheelbase=2.5, goal=None, goal_tol=1.5, max_steer=0.6):
    """纯跟踪控制律（与 ROS 无关的纯函数，便于离线回放与单测）。

    返回 ``(v, w, reached)``：线速度、角速度、是否已到终点。大转角自动降速，
    前轮转角限幅 ``max_steer``。
    """
    goal = rrt.GOAL if goal is None else goal
    x, y, yaw = pose
    if rrt.dist((x, y), goal) < goal_tol:
        return 0.0, 0.0, True
    target = rrt.lookahead_point(path, x, y, lookahead)
    if target is None:
        return 0.0, 0.0, False
    ld = math.hypot(target[0] - x, target[1] - y)
    if ld < 1e-6:
        return 0.0, 0.0, False
    alpha = wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    delta = max(-max_steer, min(max_steer, delta))
    v = v_cruise * max(0.35, 1.0 - abs(delta) / max_steer)
    w = v / wheelbase * math.tan(delta)
    return v, w, False


class RRTControlNode(Node if HAS_ROS else object):
    """规划一次，之后按路径做纯跟踪。"""

    def __init__(self, seed=3, lookahead=8.0, v=3.0, rate=20.0, wheelbase=2.5):
        super().__init__("rrt_planner_control")
        self.declare_parameter("lookahead", float(lookahead))
        self.declare_parameter("v", float(v))
        self.declare_parameter("wheelbase", float(wheelbase))
        self.lookahead = float(self.get_parameter("lookahead").value)
        self.v = float(self.get_parameter("v").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)

        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "planning/path", 10)

        planner = rrt.RRT(rrt.START, rrt.GOAL, rrt.OBSTACLES, seed=int(seed))
        raw = planner.plan()
        self.tree_nodes = len(planner.nodes)
        if raw is None:
            self.get_logger().error("RRT 未能找到可行路径")
            self.path = []
        else:
            smoothed = rrt.smooth(raw, rrt.OBSTACLES, seed=int(seed) + 2)
            # 平滑后顶点稀疏（10 个左右），必须加密到 <=0.5 m 再做纯跟踪，
            # 否则前视目标会跨过拐角、车辆从障碍内侧切进去。
            self.path = rrt.densify(smoothed, max_step=0.5)
            self.get_logger().info(
                "RRT 规划完成: 树节点 %d，原始长度 %.2f m，平滑后 %.2f m，"
                "加密后 %d 个路点（<=0.5 m）"
                % (self.tree_nodes, rrt.path_length(raw),
                   rrt.path_length(smoothed), len(self.path))
            )

        self.pose = None
        self.finished = False
        self.timer = self.create_timer(1.0 / float(rate), self._tick)
        self.get_logger().info(
            "rrt_planner_control 就绪: 前视 %.1f m, 巡航 %.1f m/s" % (self.lookahead, self.v)
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

    def _lookahead_point(self, x, y):
        """沿路径向前累积 lookahead 弧长，返回插值得到的精确前视点。"""
        return rrt.lookahead_point(self.path, x, y, self.lookahead)

    def _tick(self):
        self._publish_path()
        if self.pose is None or not self.path or self.finished:
            return

        cmd = Twist()
        v, w, reached = pure_pursuit_cmd(
            self.path, self.pose, lookahead=self.lookahead,
            v_cruise=self.v, wheelbase=self.wheelbase)
        if reached:                                # 到达目标，停车
            self.finished = True
            self.pub_cmd.publish(cmd)
            self.get_logger().info("已抵达目标 (%.1f, %.1f)，停车" % rrt.GOAL)
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
    ap = argparse.ArgumentParser(description="RRT 规划 + 纯跟踪控制节点")
    ap.add_argument("--seed", type=int, default=3, help="RRT 随机种子")
    ap.add_argument("--lookahead", type=float, default=8.0, help="纯跟踪前视距离 (m)")
    ap.add_argument("--v", type=float, default=3.0, help="巡航速度 (m/s)")
    ap.add_argument("--rate", type=float, default=20.0, help="控制频率 (Hz)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = RRTControlNode(seed=args.seed, lookahead=args.lookahead,
                          v=args.v, rate=args.rate)
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
