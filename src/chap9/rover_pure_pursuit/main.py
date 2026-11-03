#!/usr/bin/env python3
"""Self-contained 2-D kinematic rover and pure-pursuit path tracker."""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class RoverPurePursuit(Node):
    def __init__(self):
        super().__init__('rover_pure_pursuit')
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('lookahead', 0.55)
        self.speed = float(self.get_parameter('speed').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.x, self.y, self.yaw = -1.0, 0.0, 0.0
        self.path = [(2.0 * math.cos(t), 1.3 * math.sin(t)) for t in [i * 0.04 for i in range(158)]]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Float32MultiArray, '/reference_path_xy', 10)
        self.timer = self.create_timer(0.05, self.step)

    def step(self):
        target = min(self.path, key=lambda p: abs(math.hypot(p[0]-self.x, p[1]-self.y)-self.lookahead))
        bearing = math.atan2(target[1]-self.y, target[0]-self.x)
        alpha = math.atan2(math.sin(bearing-self.yaw), math.cos(bearing-self.yaw))
        angular = max(-1.2, min(1.2, 2.0*self.speed*math.sin(alpha)/max(self.lookahead, 0.1)))
        dt = 0.05
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw+angular*dt), math.cos(self.yaw+angular*dt))
        cmd = Twist(); cmd.linear.x = self.speed; cmd.angular.z = angular; self.cmd_pub.publish(cmd)
        odom = Odometry(); odom.header.stamp = self.get_clock().now().to_msg(); odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2); odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        self.odom_pub.publish(odom)
        path_msg = Float32MultiArray(); path_msg.data = [v for p in self.path for v in p]; self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args); node = RoverPurePursuit()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
