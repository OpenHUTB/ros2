#!/usr/bin/env python3
"""Clamp UAV position setpoints to a configurable 3-D geofence."""
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class GeofenceGuard(Node):
    def __init__(self):
        super().__init__('uav_geofence_guard')
        self.declare_parameter('min_xyz', [-5.0, -5.0, 0.3])
        self.declare_parameter('max_xyz', [5.0, 5.0, 3.0])
        self.minimum = list(self.get_parameter('min_xyz').value)
        self.maximum = list(self.get_parameter('max_xyz').value)
        self.sub = self.create_subscription(PointStamped, '/uav/goal_raw', self.on_goal, 10)
        self.pub = self.create_publisher(PointStamped, '/uav/goal_safe', 10)

    def on_goal(self, msg):
        requested = [msg.point.x, msg.point.y, msg.point.z]
        safe = [max(self.minimum[i], min(self.maximum[i], requested[i])) for i in range(3)]
        out = PointStamped(); out.header = msg.header
        out.point.x, out.point.y, out.point.z = safe
        self.pub.publish(out)
        if safe != requested:
            self.get_logger().warn(f'Goal clamped from {requested} to {safe}')


def main(args=None):
    rclpy.init(args=args); node = GeofenceGuard()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
