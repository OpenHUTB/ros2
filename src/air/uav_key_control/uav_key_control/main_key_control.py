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
        self.linear = 0.2
        self.angular = 0.3
        # 单独开线程读取键盘，不阻塞ROS2
        self.key_thread = threading.Thread(target=self.key_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

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
                self.msg.linear.x = self.linear
                self.msg.angular.z = 0.0
            elif key == 's':
                self.msg.linear.x = -self.linear
                self.msg.angular.z = 0.0
            elif key == 'a':
                self.msg.linear.x = 0.0
                self.msg.angular.z = self.angular
            elif key == 'd':
                self.msg.linear.x = 0.0
                self.msg.angular.z = -self.angular
            elif key == ' ':
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
            elif key == '\x03':
                break
            self.pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
