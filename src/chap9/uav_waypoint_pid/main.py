#!/usr/bin/env python3
"""Simple 3-D waypoint PID with a deterministic kinematic UAV model."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class UavWaypointPid(Node):
    def __init__(self):
        super().__init__('uav_waypoint_pid')
        self.declare_parameter('waypoints', [1.0, 0.0, 1.2, 2.0, 1.0, 1.5, 0.0, 2.0, 1.0])
        self.points = list(self.get_parameter('waypoints').value)
        self.gain = 0.8; self.max_speed = 0.45; self.pos = [0.0, 0.0, 0.0]
        self.index = 0
        self.cmd_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/uav/odom', 10)
        self.timer = self.create_timer(0.05, self.step)

    def step(self):
        if self.index >= len(self.points):
            self.cmd_pub.publish(Twist()); return
        goal = self.points[self.index:self.index+3]
        error = [goal[i]-self.pos[i] for i in range(3)]
        distance = math.sqrt(sum(e*e for e in error))
        if distance < 0.12:
            self.index += 3; return
        speed = min(self.max_speed, self.gain*distance)
        velocity = [speed*e/distance for e in error]
        dt = 0.05
        for i in range(3): self.pos[i] += velocity[i]*dt
        cmd = Twist(); cmd.linear.x, cmd.linear.y, cmd.linear.z = velocity; self.cmd_pub.publish(cmd)
        msg = Odometry(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'map'; msg.child_frame_id = 'uav'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.pos
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = UavWaypointPid()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
