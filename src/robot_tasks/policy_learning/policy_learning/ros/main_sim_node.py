"""二维运动学仿真节点：给策略节点提供 /odom，执行 /cmd_vel。

不依赖 MuJoCo / Gazebo，纯粹按差速底盘积分位姿，并在撞墙时停住。
同时发布 /map（占据栅格）和机器人位姿 Marker，方便在 rviz 里观察。
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from ..grid import H, RES, W, build_grid
from ..sim import collides, step


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_node")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("start_x", 0.85)
        self.declare_parameter("start_y", 3.05)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("topic.odom", "/odom")
        self.declare_parameter("topic.cmd", "/cmd_vel")
        self.declare_parameter("topic.map", "/map")

        self.grid = build_grid()
        self.pose = (float(self.get_parameter("start_x").value),
                     float(self.get_parameter("start_y").value),
                     float(self.get_parameter("start_yaw").value))
        self.cmd = (0.0, 0.0)
        self.collided = False

        self.create_subscription(
            Twist, str(self.get_parameter("topic.cmd").value), self.on_cmd, 10)
        self.pub_odom = self.create_publisher(
            Odometry, str(self.get_parameter("topic.odom").value), 10)
        self.pub_marker = self.create_publisher(Marker, "/robot_marker", 10)

        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("topic.map").value), map_qos)

        hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self.tick)
        self.publish_map(hz)

        self.get_logger().info(
            "仿真就绪：起点 (%.2f, %.2f)，%d x %d 栅格，%.1f Hz"
            % (self.pose[0], self.pose[1], W, H, hz))

    def on_cmd(self, msg: Twist) -> None:
        self.cmd = (float(msg.linear.x), float(msg.angular.z))

    def publish_map(self, hz: float) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = RES
        msg.info.width = W
        msg.info.height = H
        msg.info.origin.orientation.w = 1.0
        msg.data = (self.grid * 100).astype(np.int8).ravel().tolist()
        self.pub_map.publish(msg)

    def tick(self) -> None:
        if not self.collided:
            self.pose = step(self.pose, self.cmd[0], self.cmd[1], self.dt)
            if collides(self.grid, self.pose):
                self.collided = True
                self.pose = (self.pose[0], self.pose[1], self.pose[2])
                self.get_logger().warn("机器人撞到障碍，已停住")
                self.cmd = (0.0, 0.0)

        self.publish_odom()
        self.publish_marker()

    def publish_odom(self) -> None:
        m = Odometry()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.child_frame_id = "base_link"
        m.pose.pose.position.x = self.pose[0]
        m.pose.pose.position.y = self.pose[1]
        yaw = self.pose[2]
        m.pose.pose.orientation.z = math.sin(yaw * 0.5)
        m.pose.pose.orientation.w = math.cos(yaw * 0.5)
        m.twist.twist.linear.x = self.cmd[0]
        m.twist.twist.angular.z = self.cmd[1]
        self.pub_odom.publish(m)

    def publish_marker(self) -> None:
        mk = Marker()
        mk.header.frame_id = "map"
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.type = Marker.CYLINDER
        mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = 0.36
        mk.scale.z = 0.2
        mk.pose.position.x = self.pose[0]
        mk.pose.position.y = self.pose[1]
        mk.pose.position.z = 0.1
        mk.pose.orientation.z = math.sin(self.pose[2] * 0.5)
        mk.pose.orientation.w = math.cos(self.pose[2] * 0.5)
        mk.color.r, mk.color.g, mk.color.b = 0.1, 0.8, 0.3
        mk.color.a = 0.9
        self.pub_marker.publish(mk)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
