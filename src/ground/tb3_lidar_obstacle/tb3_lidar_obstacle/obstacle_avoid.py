# -*- coding: utf-8 -*-
"""ROS 2 node wrapper around the pure-Python avoidance logic."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from .avoid_logic import SCAN_TIMEOUT_S, STOP, scan_to_decision


class AvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)

        self.declare_parameter('safe_dist', 0.5)
        self.declare_parameter('scan_timeout', SCAN_TIMEOUT_S)
        self.safe_dist = float(self.get_parameter('safe_dist').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)

        self.last_scan_time = self.get_clock().now()
        self.watchdog_warned = False
        # 0.1 s granularity: stop within ~0.5 s of /scan going silent.
        self.timer = self.create_timer(0.1, self.watchdog_callback)

    def _publish(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        self.watchdog_warned = False
        linear_x, angular_z, state = scan_to_decision(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.safe_dist)
        self.get_logger().info(
            'scan: state=%s front_sector=%.2f m -> v=%.2f w=%.2f'
            % (state,
               self._front_distance(msg),
               linear_x, angular_z))
        self._publish(linear_x, angular_z)

    def _front_distance(self, msg):
        from .avoid_logic import FRONT_HALF_ANGLE_DEG, sector_min
        d = sector_min(msg.ranges, msg.angle_min, msg.angle_increment,
                       0.0, FRONT_HALF_ANGLE_DEG,
                       msg.range_min, msg.range_max)
        return d if math.isfinite(d) else 10.0

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed < self.scan_timeout:
            return
        self._publish(*STOP[:2])
        if not self.watchdog_warned:
            self.get_logger().warn(
                'no /scan for %.2f s (> %.2f s): published zero velocity'
                % (elapsed, self.scan_timeout))
            self.watchdog_warned = True

    def destroy_node(self):
        self._publish(*STOP[:2])
        return super().destroy_node()


def main():
    rclpy.init()
    node = AvoidNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
