"""Publish a ROS 2 scan and odometry stream from the built-in roundabout sim."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .dataset import rule_label
from .navigation import raycast_circles
from .simulator import RoundaboutEnv


class RoundaboutSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("roundabout_simulator")
        self.declare_parameter("scan_rays", 25)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("period_s", 0.1)
        self.rays = int(self.get_parameter("scan_rays").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.env = RoundaboutEnv()
        self.env.reset()
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.collision_pub = self.create_publisher(Bool, "/simulation/collision", 10)
        self.finished_pub = self.create_publisher(Bool, "/simulation/route_complete", 10)
        self.done_pub = self.create_publisher(Bool, "/simulation/done", 10)
        self.timer = self.create_timer(float(self.get_parameter("period_s").value), self._tick)
        self._final_state_published = False
        self.finished_at = None

    def _tick(self) -> None:
        if not self.env.done:
            oncoming_x, oncoming_y = self.env._npc_state()
            action = rule_label(self.env.lateral, oncoming_x, oncoming_y)
            self.env.step(action)

        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(stamp)
        self._publish_scan(stamp)
        self.collision_pub.publish(Bool(data=self.env.crashed))
        self.finished_pub.publish(Bool(data=self.env.finished))
        self.done_pub.publish(Bool(data=self.env.done))
        if self.env.done and not self._final_state_published:
            self._final_state_published = True
            self.timer.cancel()
            self.finished_at = time.monotonic()
            self.get_logger().info(
                f"simulation finished: completed={self.env.finished}, collision={self.env.crashed}, "
                f"time_s={self.env.t:.1f}"
            )

    def _publish_odometry(self, stamp) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self.env.x
        message.pose.pose.position.y = self.env.lateral
        message.pose.pose.orientation = Quaternion(w=1.0)
        message.twist.twist.linear.x = self.env.v
        self.odom_pub.publish(message)

    def _publish_scan(self, stamp) -> None:
        oncoming_x, oncoming_y = self.env._npc_state()
        origin = (self.env.x, self.env.lateral)
        circle = (self.env.x + oncoming_x, self.env.lateral + oncoming_y, 1.1)
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.angle_min = -math.pi / 2.0
        message.angle_max = math.pi / 2.0
        message.angle_increment = math.pi / (self.rays - 1)
        message.range_min = 0.05
        message.range_max = self.range_max
        message.ranges = raycast_circles(origin, 0.0, [circle], self.rays, self.range_max)
        self.scan_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoundaboutSimulatorNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.finished_at is not None and time.monotonic() - node.finished_at >= 0.5:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
