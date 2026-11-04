"""ROS 2 节点：订阅位姿与目标，用训练好的策略网络直接输出速度指令。"""
from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ..encoder import N_OBS, encode
from ..expert import LOOKAHEAD, steer
from ..grid import to_cell
from ..mlp import PolicyMLP
from ..value import descend_path, distance_map


def _share_dir() -> str:
    """安装后从包 share 目录取权重；源码直接运行时回落到包目录。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("policy_learning")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.dirname(os.path.dirname(here))


def default_model_path() -> str:
    return os.path.join(_share_dir(), "data", "policy.npz")


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("policy_node")

        self.declare_parameter("model", "")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("topic.odom", "/odom")
        self.declare_parameter("topic.goal", "/goal_pose")
        self.declare_parameter("topic.cmd", "/cmd_vel")
        self.declare_parameter("topic.plan", "/plan")
        self.declare_parameter("publish_plan", True)

        model_path = str(self.get_parameter("model").value) or default_model_path()
        if not os.path.isfile(model_path):
            raise RuntimeError("找不到策略权重：%s" % model_path)
        self.net = PolicyMLP.load(model_path)

        from ..grid import planning_grid
        self.grid = planning_grid()

        self.pose = None
        self.goal = None
        self.dist = None
        self.last_cmd = (0.0, 0.0)
        self.arrived = False
        self._k = 0

        odom_topic = str(self.get_parameter("topic.odom").value)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter("topic.goal").value),
            self.on_goal, 10)

        self.pub_cmd = self.create_publisher(
            Twist, str(self.get_parameter("topic.cmd").value), 10)
        self.pub_plan = self.create_publisher(
            PathMsg, str(self.get_parameter("topic.plan").value), 10)

        hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / hz, self.tick)

        self.get_logger().info(
            "策略已加载：%s（%d 参数，%d 维输入），控制频率 %.1f Hz"
            % (os.path.basename(model_path), self.net.n_params(),
               N_OBS, hz))

    # ---------- 回调 ----------
    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        i, j = to_cell(self.grid, p.x, p.y)
        if self.grid[j, i]:
            self.get_logger().warn(
                "目标点 (%.2f, %.2f) 落在障碍内，忽略" % (p.x, p.y))
            return
        dist = distance_map(self.grid, (p.x, p.y))
        if not np.isfinite(dist[j, i]):
            self.get_logger().warn("目标点不可达，忽略")
            return
        self.dist = dist
        self.goal = (p.x, p.y)
        self.arrived = False          # 新目标要清掉上一轮的到达状态
        self._k = 0
        if bool(self.get_parameter("publish_plan").value):
            self.publish_plan()
        self.get_logger().info("收到目标 (%.2f, %.2f)" % (p.x, p.y))

    # ---------- 主循环 ----------
    def publish_plan(self) -> None:
        if self.goal is None or self.dist is None or self.pose is None:
            return
        path = descend_path(self.dist, self.grid, self.pose[:2])
        if path is None:
            return
        msg = PathMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def publish(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub_cmd.publish(t)
        self.last_cmd = (float(v), float(w))

    def tick(self) -> None:
        if self.pose is None or self.goal is None or self.dist is None:
            self.publish(0.0, 0.0)
            return

        self._k += 1
        # rviz 的 Path 是 Volatile QoS，周期性重发，避免晚订阅漏掉
        if self._k % 10 == 0 and bool(self.get_parameter("publish_plan").value):
            self.publish_plan()

        if self.arrived:
            self.publish(0.0, 0.0)
            return

        if math.hypot(self.pose[0] - self.goal[0],
                      self.pose[1] - self.goal[1]) < 0.2:
            self.arrived = True
            self.publish(0.0, 0.0)
            self.publish_plan()
            self.get_logger().info("已到达目标")
            return

        obs = encode(self.grid, self.pose, self.goal, self.dist)
        p = self.net.predict(obs)
        v, w = steer(float(p[0]) * LOOKAHEAD, float(p[1]) * LOOKAHEAD)
        self.publish(v, w)

    def stop(self) -> None:
        try:
            self.publish(0.0, 0.0)
        except Exception:
            pass


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
