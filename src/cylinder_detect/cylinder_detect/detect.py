#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CylinderDetectNode(Node):
    def __init__(self):
        super().__init__('cylinder_node')
        self.get_logger().info("圆柱检测节点启动成功！")

def main():
    rclpy.init()
    node = CylinderDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
