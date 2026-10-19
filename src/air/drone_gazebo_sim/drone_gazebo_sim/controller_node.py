#!/usr/bin/env python3
"""无人机跟踪控制器
订阅感知节点的目标位置，用PID控制无人机飞向红色目标
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.sub = self.create_subscription(
            Float32MultiArray, '/drone/target_pos', self.target_cb, 10)
        self.pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.kp_yaw = 0.8   # 水平比例
        self.kp_forward = 0.5  # 前进比例
        self.target_area = 0.15  # 目标占比达到此值即悬停
        self.get_logger().info('无人机目标跟踪控制器已启动')

    def target_cb(self, msg):
        out = Twist()
        if len(msg.data) < 2 or msg.data[0] > 900:
            # 没找到目标，原地悬停
            out.linear.x = out.linear.y = out.linear.z = 0.0
        else:
            cx, area = msg.data[0], msg.data[1]
            # 水平对齐
            out.angular.z = -self.kp_yaw * cx
            # 前进/后退
            if area < self.target_area:
                out.linear.x = self.kp_forward * (self.target_area - area)
            else:
                out.linear.x = 0.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
