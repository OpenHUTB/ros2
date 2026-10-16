#!/usr/bin/env python3
"""人形机器人控制器：PID根据感知结果追踪目标"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.sub = self.create_subscription(Point, '/humanoid/target_pos', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.kp_x = self.declare_parameter('kp_x', 0.5).value
        self.kp_y = self.declare_parameter('kp_y', 0.3).value
        self.deadzone = self.declare_parameter('deadzone', 0.05).value
        self.max_speed = self.declare_parameter('max_speed', 0.2).value

    def cb(self, msg):
        if msg.z < 0.5: return
        t = Twist()
        if abs(msg.y) > self.deadzone:
            t.linear.x = max(-self.max_speed, min(self.max_speed, -self.kp_x * msg.y))
        if abs(msg.x) > self.deadzone:
            t.angular.z = max(-self.max_speed, min(self.max_speed, -self.kp_y * msg.x))
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
