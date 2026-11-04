#!/usr/bin/env python3
"""Low-order 6-DOF ROV motion model driven by geometry_msgs/Twist."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RovTeleopModel(Node):
    def __init__(self):
        super().__init__('rov_6dof_teleop')
        self.declare_parameter('damping', 0.8)
        self.damping = float(self.get_parameter('damping').value)
        self.command = [0.0]*6; self.velocity = [0.0]*6
        self.position = [0.0]*3; self.rpy = [0.0]*3
        self.sub = self.create_subscription(Twist, '/rov/cmd_vel', self.on_cmd, 10)
        self.pub = self.create_publisher(Odometry, '/rov/odom', 10)
        self.timer = self.create_timer(0.05, self.step)

    def on_cmd(self, msg):
        self.command = [msg.linear.x, msg.linear.y, msg.linear.z,
                        msg.angular.x, msg.angular.y, msg.angular.z]

    def step(self):
        dt = 0.05
        for i in range(6):
            self.velocity[i] += (self.command[i]-self.damping*self.velocity[i])*dt
        for i in range(3):
            self.position[i] += self.velocity[i]*dt
            self.rpy[i] += self.velocity[i+3]*dt
        cy, sy = math.cos(self.rpy[2]/2), math.sin(self.rpy[2]/2)
        cp, sp = math.cos(self.rpy[1]/2), math.sin(self.rpy[1]/2)
        cr, sr = math.cos(self.rpy[0]/2), math.sin(self.rpy[0]/2)
        msg = Odometry(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'; msg.child_frame_id = 'rov/base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.position
        q = msg.pose.pose.orientation
        q.w = cr*cp*cy + sr*sp*sy; q.x = sr*cp*cy - cr*sp*sy
        q.y = cr*sp*cy + sr*cp*sy; q.z = cr*cp*sy - sr*sp*cy
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = RovTeleopModel()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
