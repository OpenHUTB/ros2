#!/usr/bin/env python3
"""键盘控制节点：向 /cmd_vel 发布 geometry_msgs/Twist 消息，控制 Gazebo 中的差速小车。"""
import select
import sys
import time
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    """读取键盘按键，持续向 /cmd_vel 发布速度指令。"""

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.speed = 0.5    # 线速度 m/s
        self.turn = 1.0     # 角速度 rad/s

        # 保存终端原始设置，退出时恢复
        self.term_settings = termios.tcgetattr(sys.stdin)
        self.print_usage()

    def print_usage(self):
        self.get_logger().info('键盘控制:')
        self.get_logger().info('  w/x  前进/后退')
        self.get_logger().info('  a/d  左转/右转')
        self.get_logger().info('  s    停止')
        self.get_logger().info('  +/-  增大/减小线速度')
        self.get_logger().info('  q    退出')

    def get_key(self):
        """非阻塞读取单个按键，无输入时返回空串。"""
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = ''
        if ready:
            key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.term_settings)
        return key

    def handle_key(self, key):
        """根据按键更新目标速度。"""
        if key == 'w':
            self.linear_x = self.speed
            self.angular_z = 0.0
        elif key == 'x':
            self.linear_x = -self.speed
            self.angular_z = 0.0
        elif key == 'a':
            self.linear_x = 0.0
            self.angular_z = self.turn
        elif key == 'd':
            self.linear_x = 0.0
            self.angular_z = -self.turn
        elif key == 's':
            self.linear_x = 0.0
            self.angular_z = 0.0
        elif key == '+':
            self.speed = min(2.0, self.speed + 0.1)
            self.get_logger().info(f'线速度: {self.speed:.2f} m/s')
        elif key == '-':
            self.speed = max(0.0, self.speed - 0.1)
            self.get_logger().info(f'线速度: {self.speed:.2f} m/s')

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z
        self.publisher.publish(twist)

    def spin_teleop(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in ('q', '\x03'):
                    self.get_logger().info('退出。')
                    break
                self.handle_key(key)
                # 以约 20Hz 持续发布当前速度指令
                self.publish_cmd()
                time.sleep(0.05)
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.term_settings)
            # 退出前停止小车
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.spin_teleop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
