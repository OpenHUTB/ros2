import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios
import threading

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_uav_control')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("WASD控制：W前进 S后退 A左转 D右转，空格停止，Ctrl+C退出")
        self.msg = Twist()
        # 定义速度上限（边界参数）
        self.MAX_LINEAR = 0.2
        self.MAX_ANGULAR = 0.3

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def key_loop(self):
        while True:
            key = self.get_key()
            if key == 'w':
                linear_x = self.MAX_LINEAR
                angular_z = 0.0
            elif key == 's':
                linear_x = -self.MAX_LINEAR
                angular_z = 0.0
            elif key == 'a':
                linear_x = 0.0
                angular_z = self.MAX_ANGULAR
            elif key == 'd':
                linear_x = 0.0
                angular_z = -self.MAX_ANGULAR
            elif key == ' ':
                linear_x = 0.0
                angular_z = 0.0
            elif key == '\x03':
                break
            else:
                # 其他按键，保持静止
                linear_x = 0.0
                angular_z = 0.0

            # 边界钳位保护
            linear_x = max(-self.MAX_LINEAR, min(linear_x, self.MAX_LINEAR))
            angular_z = max(-self.MAX_ANGULAR, min(angular_z, self.MAX_ANGULAR))

            self.msg.linear.x = linear_x
            self.msg.angular.z = angular_z
            self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
