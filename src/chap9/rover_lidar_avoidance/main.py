#!/usr/bin/env python3
"""Reactive differential-drive rover using a LaserScan and /cmd_vel."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('rover_lidar_avoidance')
        self.declare_parameter('safe_distance', 0.65)
        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.8)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.left_clearance = float('inf')
        self.right_clearance = float('inf')
        self.front_clearance = float('inf')
        self.scan_ready = False
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_step)
        self.get_logger().info('Waiting for /scan; publishing safe /cmd_vel commands.')

    @staticmethod
    def sector_min(msg, low_deg, high_deg):
        low, high = math.radians(low_deg), math.radians(high_deg)
        values = []
        for i, raw in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if low <= angle <= high and math.isfinite(raw):
                if msg.range_min <= raw <= msg.range_max:
                    values.append(raw)
        return min(values, default=float('inf'))

    def on_scan(self, msg):
        self.front_clearance = self.sector_min(msg, -18, 18)
        self.left_clearance = self.sector_min(msg, 25, 85)
        self.right_clearance = self.sector_min(msg, -85, -25)
        self.scan_ready = True

    def control_step(self):
        cmd = Twist()
        if self.scan_ready:
            if self.front_clearance < self.safe_distance:
                cmd.angular.z = self.turn_speed if self.left_clearance >= self.right_clearance else -self.turn_speed
            else:
                cmd.linear.x = self.forward_speed
                if min(self.left_clearance, self.right_clearance) < self.safe_distance:
                    cmd.angular.z = 0.25 if self.left_clearance > self.right_clearance else -0.25
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
