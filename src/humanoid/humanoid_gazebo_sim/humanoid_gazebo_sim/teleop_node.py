#!/usr/bin/env python3
"""人形机器人键盘遥控节点
按键: w/s=前进后退 a/d=左右 q/e=左转右转 空格=停止
"""
import sys, tty, termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HumanoidTeleop(Node):
    def __init__(self):
        super().__init__('humanoid_teleop')
        self.pub = self.create_publisher(Twist, '/humanoid/cmd_vel', 10)
        self.vx = self.declare_parameter('vx_max', 0.3).value
        self.vy = self.declare_parameter('vy_max', 0.2).value
        self.yaw = self.declare_parameter('yaw_max', 0.4).value
        self.get_logger().info('人形机器人键盘控制: w/s=前后 a/d=左右 q/e=转向 空格=停止')

    def run(self):
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                msg = Twist()
                if key == 'w': msg.linear.x = self.vx
                elif key == 's': msg.linear.x = -self.vx
                elif key == 'a': msg.linear.y = self.vy
                elif key == 'd': msg.linear.y = -self.vy
                elif key == 'q': msg.angular.z = self.yaw
                elif key == 'e': msg.angular.z = -self.yaw
                elif key == ' ': pass
                elif key == '\x03': break
                self.pub.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
