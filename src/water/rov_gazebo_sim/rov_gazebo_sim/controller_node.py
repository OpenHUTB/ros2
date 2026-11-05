#!/usr/bin/env python3
"""水下ROV控制器：PID根据感知结果追踪目标"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


class RovController(Node):
    def __init__(self):
        super().__init__('rov_controller')
        self.sub = self.create_subscription(
            Point, '/rov/target_pos', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.kp_x = self.declare_parameter('kp_x', 0.5).value
        self.kp_y = self.declare_parameter('kp_y', 0.3).value
        self.deadzone = self.declare_parameter('deadzone', 0.05).value
        self.max_speed = self.declare_parameter('max_speed', 0.2).value
        self.get_logger().info('ROV控制器已启动')

    def cb(self, msg):
        if msg.z < 0.5:
            return
        twist = Twist()
        # x方向（前后）：目标在画面中心偏上就前进
        if abs(msg.y) > self.deadzone:
            twist.linear.x = max(-self.max_speed,
                                  min(self.max_speed, -self.kp_x * msg.y))
        # y方向（左右）：根据目标x偏移修正
        if abs(msg.x) > self.deadzone:
            twist.linear.y = max(-self.max_speed,
                                  min(self.max_speed, -self.kp_y * msg.x))
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RovController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
