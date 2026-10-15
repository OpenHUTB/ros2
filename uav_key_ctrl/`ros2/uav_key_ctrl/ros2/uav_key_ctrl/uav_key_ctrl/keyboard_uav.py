#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios

class KeyboardUAVControl(Node):
    def __init__(self):
        super().__init__('keyboard_uav_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.get_logger().info("无人机键盘控制已启动！\nw:前进 s:后退 a:左转 d:右转\n空格:停止，按Ctrl+C退出")

    def getKey(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        while True:
            key = self.getKey()
            if key == 'w':
                self.twist.linear.x = 0.2
            elif key == 's':
                self.twist.linear.x = -0.2
            elif key == 'a':
                self.twist.angular.z = 0.2
            elif key == 'd':
                self.twist.angular.z = -0.2
            elif key == ' ':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            elif key == '\x03':
                break
            self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = KeyboardUAVControl()
    try:
        node.run()
    except KeyboardInterrupt:
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.pub.publish(node.twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
