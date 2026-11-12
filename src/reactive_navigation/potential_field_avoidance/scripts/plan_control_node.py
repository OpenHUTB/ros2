#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""APF 势场导航 + 跟踪控制节点。

启动时用 ``apf.navigate`` 离线执行一次势场下降（含切向滑行逃逸），得到参考
轨迹并以 ``nav_msgs/Path`` 发布到 ``planning/path``（RViz 可直接可视化）；
随后持续订阅 ``odom``，按里程计反馈跟踪这条轨迹，以 ``geometry_msgs/Twist``
发布到 ``cmd_vel``。

为什么不"每步在线算梯度"：``apf.step_command`` 提供了单步决策接口（供教学
与二次开发），但逃逸（切向滑行）是有状态的成段行为，在线逐步重算会让节点
与离线结果不一致。所以这里与 rrt_path_planning 保持同一模式：**离线执行
算法得到参考轨迹，控制端只做反馈跟踪**，两条链路的数值都可以被
``tests/test_apf.py`` 锁死。

完整闭环（配合 sim_node）：

    sim_node --/odom--> plan_control_node --/cmd_vel--> sim_node
    plan_control_node --/planning/path--> RViz / 下游

启动：

    ros2 launch potential_field_avoidance potential_field_avoidance.launch.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import apf  # noqa: E402

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


def track_cmd(path, pose, idx, speed, arrive_r=0.55, w_k=2.5, w_max=2.0,
              goal=None, goal_tol=0.8):
    """反馈跟踪（与 ROS 无关的纯函数，便于离线回放与单测）。

    从 ``idx`` 起把游标推进到路径上离当前位置最近（允许越过）的点，朝它
    开车；大航向误差自动减速。返回 ``(v, w, new_idx, done)``。
    """
    x, y, yaw = pose
    goal = path[-1] if goal is None else goal
    if math.hypot(x - goal[0], y - goal[1]) < goal_tol:
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


class APFControlNode(Node if HAS_ROS else object):
    """势场导航：离线求轨迹一次，之后反馈跟踪。"""

    def __init__(self, v=2.0, rate=20.0, trap=False, escape=True):
        super().__init__("apf_planner_control")
        self.declare_parameter("v", float(v))
        self.declare_parameter("trap", bool(trap))
        self.declare_parameter("escape", bool(escape))
        self.v = float(self.get_parameter("v").value)
        self.trap = bool(self.get_parameter("trap").value)
        self.escape = bool(self.get_parameter("escape").value)

        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "planning/path", 10)

        if self.trap:                       # 凹形陷阱演示（局部极小 + 逃逸）
            start, goal, obstacles = apf.TRAP_START, apf.TRAP_GOAL, apf.TRAP_OBSTACLES
        else:
            start, goal, obstacles = apf.START, apf.GOAL, apf.OBSTACLES
        res = apf.navigate(start, goal, obstacles, escape=self.escape)
        # 不可达（卡死在局部极小）时没有可跟踪的参考轨迹：清空路径，
        # _tick 的空路径卫门会拦住 cmd_vel——机器人保持静止（评审 #1400
        # 复核：旧版把残缺路径也交给跟踪器，车会一直绕着路径末端转）
        self.path = res["path"] if res["success"] else []
        self.goal = goal
        self.get_logger().info(
            "APF 参考轨迹: %s, 长度 %.1f m, %d 步, 逃逸 %d 次, 最小间隙 %.2f m%s"
            % ("成功" if res["success"] else "未到达", res["length"], res["steps"],
               res["escapes"], res["min_clear"],
               "" if res["stuck"] is None else ", 卡点 (%.1f, %.1f)" % res["stuck"])
        )
        if not res["success"]:
            self.get_logger().warn("势场法在局部极小卡死（escape=%s）：无参考"
                                    "轨迹可跟踪，已清空路径、不发布 cmd_vel，"
                                    "节点保持静止" % self.escape)

        self.pose = None
        self.idx = 0
        self.finished = False
        self.timer = self.create_timer(1.0 / float(rate), self._tick)
        self.get_logger().info("apf_planner_control 就绪: 巡航 %.1f m/s" % self.v)

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
        if self.pose is None or not self.path or self.finished:
            return

        cmd = Twist()
        v, w, self.idx, reached = track_cmd(self.path, self.pose, self.idx,
                                            speed=self.v, goal=self.goal)
        if reached:                                # 到达目标，停车
            self.finished = True
            self.pub_cmd.publish(cmd)
            self.get_logger().info("已抵达目标 (%.1f, %.1f)，停车" % self.goal)
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
    ap = argparse.ArgumentParser(description="APF 势场导航 + 跟踪控制节点")
    ap.add_argument("--v", type=float, default=2.0, help="巡航速度 (m/s)")
    ap.add_argument("--rate", type=float, default=20.0, help="控制频率 (Hz)")
    ap.add_argument("--trap", action="store_true",
                    help="使用凹形陷阱场景（演示局部极小与逃逸）")
    ap.add_argument("--no-escape", action="store_true", help="关闭切向滑行逃逸")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_cli(argv)
    if not HAS_ROS:
        print("[提示] 未检测到 rclpy，本节点需要 ROS 2 环境才能运行。")
        print("       纯离线演示请运行: ./main.sh")
        return 1
    rclpy.init(args=list(sys.argv))
    node = APFControlNode(v=args.v, rate=args.rate,
                          trap=args.trap, escape=not args.no_escape)
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
