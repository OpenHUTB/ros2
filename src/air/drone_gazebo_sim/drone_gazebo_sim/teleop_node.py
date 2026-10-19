#!/usr/bin/env python3
"""无人机键盘遥操作节点
按键映射：
  w/s : 前后 (vx)
  a/d : 左右 (vy)
  q/e : 上升/下降 (vz)
  1/2 : 左旋/右旋 (yaw rate)
  空格: 紧急悬停
  t   : 起飞 (起飞高度1m)
  l   : 降落
"""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.vx = self.declare_parameter('vx_max', 0.5).value
        self.vy = self.declare_parameter('vy_max', 0.5).value
        self.vz = self.declare_parameter('vz_max', 0.3).value
        self.yaw = self.declare_parameter('yaw_max', 0.5).value
        self.get_logger().info(
            '无人机键盘控制已启动。\n'
            '  w/s=前后  a/d=左右  q/e=升降\n'
            '  1/2=左旋右旋  空格=悬停  t=起飞  l=降落')

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
                elif key == ' ':
                    msg.linear.x = msg.linear.y = msg.linear.z = 0.0
                elif key == 't':
                    msg.linear.z = self.vz
                    self.get_logger().info('起飞...')
                elif key == 'l':
                    msg.linear.z = -self.vz
                    self.get_logger().info('降落...')
                elif key == '\x03':  # Ctrl+C
                    break
                self.pub.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = DroneTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
