import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class DisturbanceDrive(Node):
    def __init__(self):
        super().__init__('disturbance_drive')
       self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.base_speed = 0.2

    def timer_callback(self):
        msg = Twist()
        disturb = random.uniform(-0.08, 0.08)
        msg.linear.x = self.base_speed
        msg.angular.z = disturb
        self.pub.publish(msg)
        self.get_logger().info(f"speed:{self.base_speed:.2f}, disturb_angle:{disturb:.3f}")

def main():
    rclpy.init()
    node = DisturbanceDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
