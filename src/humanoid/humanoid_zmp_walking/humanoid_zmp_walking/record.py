import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('record_node')
    node.get_logger().info('Record start')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
