#!/usr/bin/env python3
"""水下ROV键盘遥控节点

按键映射:
  w/s : 前后 (vx)
  a/d : 左右 (vy)
  q/e : 下潜/上浮 (vz)
  1/2 : 左旋/右旋 (yaw rate)
  空格 : 紧急悬停
  t : 下潜1m
  l : 上浮1m
"""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RovTeleop(Node):
    def __init__(self):
        super().__init__('rov_teleop')
        self.pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.vx = self.declare_parameter('vx_max', 0.3).value
        self.vy = self.declare_parameter('vy_max', 0.3).value
        self.vz = self.declare_parameter('vz_max', 0.2).value
        self.yaw = self.declare_parameter('yaw_max', 0.3).value
        self.get_logger().info(
            '水下ROV键盘控制已启动。\n'
            '  w/s=前后  a/d=左右  q/e=下潜/上浮\n'
            '  1/2=左旋右旋  空格=悬停  t=下潜  l=上浮')

    def run(self):
        old = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == '':
                    break
                msg = Twist()
                if key == 'w':
                    msg.linear.x = self.vx
                elif key == 's':
                    msg.linear.x = -self.vx
                elif key == 'a':
                    msg.linear.y = self.vy
                elif key == 'd':
                    msg.linear.y = -self.vy
                elif key == 'q':
                    msg.linear.z = self.vz
                elif key == 'e':
                    msg.linear.z = -self.vz
                elif key == '1':
                    msg.angular.z = self.yaw
                elif key == '2':
                    msg.angular.z = -self.yaw
                elif key == 't':
                    msg.linear.z = self.vz
                elif key == 'l':
                    msg.linear.z = -self.vz
                elif key == ' ':
                    pass  # all zeros = hover
                elif key == '\x03':
                    break
                self.pub.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = RovTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
