#!/usr/bin/env python3
"""自动巡航演示节点：持续发布固定速度指令，让小车沿圆形轨迹行驶。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoDrive(Node):
    """以固定线速度 + 角速度发布 /cmd_vel，实现圆形巡航轨迹。"""

    def __init__(self):
        super().__init__('auto_drive')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.get_logger().info('自动巡航已启动：小车沿圆形轨迹行驶')

    def publish_cmd(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.5   # 持续转向 -> 圆形轨迹
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
