import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        msg = Twist()
        while rclpy.ok():
            k = self.get_key()
            if k == 'w':
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            elif k == 's':
                msg.linear.x = -self.linear_speed
                msg.angular.z = 0.0
            elif k == 'a':
                msg.linear.x = 0.0
                msg.angular.z = self.angular_speed
            elif k == 'd':
                msg.linear.x = 0.0
                msg.angular.z = -self.angular_speed
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.pub.publish(msg)
            if k == 'q':
                break

def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
