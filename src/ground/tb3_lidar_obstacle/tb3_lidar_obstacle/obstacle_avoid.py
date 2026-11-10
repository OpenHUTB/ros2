import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class AvoidNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoid')
        # 速度指令发布
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 适配激光雷达QoS
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 订阅激光雷达数据
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.twist_msg = Twist()
        self.safe_dist = 0.5

    def scan_callback(self, msg: LaserScan):
        front_dist = msg.ranges[0]

        # 过滤inf、nan无效数据
        if math.isinf(front_dist) or math.isnan(front_dist):
            front_dist = 10.0

        self.get_logger().info(f"前方距离: {front_dist:.2f} m")

        # 避障判断
        if front_dist < self.safe_dist:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.3
        else:
            self.twist_msg.linear.x = 0.2
            self.twist_msg.angular.z = 0.0

        self.pub.publish(self.twist_msg)


def main():
    rclpy.init()
    node = AvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
