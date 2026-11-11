#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SonarSlamNode(Node):
    def __init__(self):
        super().__init__('sonar_slam_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/rov/pose',
            self.pose_callback,
            10)
        self.get_logger().info('声呐建图与避障节点已启动，正在监听 /rov/pose...')

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f'接收到 ROV 位置更新 -> [x: {x:.2f}, y: {y:.2f}, z: {z:.2f}]', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = SonarSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()