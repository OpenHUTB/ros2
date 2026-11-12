"""ROS 2 节点：禁飞区膨胀 + 可见图最短绕行，逐点输出安全目标。

启动时按 config/params.yaml 规划一次，之后按航点顺序执行；收到新目标时重新规划。
若当前位置落在膨胀区内，先取最近边界点脱困。

话题：
    订阅  /uav/odom            nav_msgs/Odometry
    订阅  /uav/goal            geometry_msgs/PoseStamped
    发布  /uav/safe_goal       geometry_msgs/PoseStamped
    发布  /uav/geofence_state  std_msgs/String（诊断）
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from uav_geofence_bypass import core

DEFAULT_PARAMS = {
    "margin": 0.6,
    "accept_radius": 0.3,
    "log_period": 1.0,
    "start": [0.0, 0.0],
    "goal": [15.0, 0.0],
    "zones": [
        [5.0, -1.2, 7.0, -1.2, 7.0, 1.0, 5.0, 1.0],
        [10.0, -1.0, 12.0, -1.0, 12.0, 1.2, 10.0, 1.2],
    ],
}


def load_params(path):
    """读取 yaml 中 uav_geofence_bypass 节点下的 ros__parameters。"""
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data.get("uav_geofence_bypass", {}).get("ros__parameters", {}) or {}


def parse_zones(raw):
    zones = []
    for flat in raw:
        values = [float(v) for v in flat]
        if len(values) < 6 or len(values) % 2 != 0:
            raise ValueError("禁飞区顶点需要成对的 x,y，至少 3 个顶点")
        zones.append([(values[i], values[i + 1]) for i in range(0, len(values), 2)])
    return zones


class GeofenceBypassNode(Node):
    def __init__(self):
        super().__init__("uav_geofence_bypass")
        self.declare_parameter("params_file", "")

        cfg = dict(DEFAULT_PARAMS)
        path = str(self.get_parameter("params_file").value or "")
        if path:
            if os.path.exists(path):
                cfg.update(load_params(path))
                self.get_logger().info("loaded params from %s" % path)
            else:
                self.get_logger().warn("params_file not found: %s, using defaults" % path)
        else:
            self.get_logger().warn("params_file empty, using defaults")

        self.margin = float(cfg["margin"])
        self.accept_radius = float(cfg["accept_radius"])
        self.log_period = float(cfg["log_period"])
        self.start = tuple(float(v) for v in cfg["start"])
        self.goal = tuple(float(v) for v in cfg["goal"])
        self.zones = parse_zones(cfg["zones"])

        self.pose = self.start
        self.plan = [self.start, self.goal]
        self.blocked = False
        self.index = 1
        self.last_log = 0.0

        self.pub = self.create_publisher(PoseStamped, "/uav/safe_goal", 10)
        self.state = self.create_publisher(String, "/uav/geofence_state", 10)
        self.create_subscription(Odometry, "/uav/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/uav/goal", self.on_goal, 10)
        self.create_timer(0.2, self.tick)
        self.replan()
        self.get_logger().info(
            "geofence bypass: %d zones margin=%.2f plan_points=%d blocked=%s"
            % (len(self.zones), self.margin, len(self.plan), self.blocked))

    def replan(self):
        self.plan, self.blocked = core.plan_path(self.pose, self.goal, self.zones, self.margin)
        self.index = min(1, len(self.plan) - 1)

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.replan()
        self.get_logger().info("new goal (%.2f, %.2f)" % self.goal)

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def tick(self):
        while self.index < len(self.plan) - 1:
            wx, wy = self.plan[self.index]
            if math.hypot(wx - self.pose[0], wy - self.pose[1]) <= self.accept_radius:
                self.index += 1
            else:
                break
        wp = self.plan[self.index]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = wp[0]
        msg.pose.position.y = wp[1]
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        mode = "direct" if len(self.plan) == 2 and not self.blocked else "bypass"
        self.state.publish(String(data="%.2f,%.2f,%s,%d" % (wp[0], wp[1], mode, self.index)))

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log >= self.log_period:
            self.last_log = now
            self.get_logger().info(
                "pos=(%.2f,%.2f) mode=%s wp=%d/%d target=(%.2f,%.2f) dist=%.2f"
                % (self.pose[0], self.pose[1], mode, self.index, len(self.plan),
                   wp[0], wp[1], math.hypot(wp[0] - self.pose[0], wp[1] - self.pose[1])))


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceBypassNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
