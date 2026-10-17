#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .policy import MlpPolicy, action_to_velocity

class PpoNavNode(Node):
    def __init__(self):
        super().__init__('ppo_nav_node')
        self.get_logger().info("PPO 导航节点已启动！")
        self.policy = MlpPolicy()
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.latest_odom = None

    def odom_cb(self, msg):
        self.latest_odom = msg

    def timer_cb(self):
        if self.latest_odom is None:
            self.get_logger().info("等待 /uav/odom 数据...", throttle_duration_sec=2.0)
            return
        obs = np.random.randn(22)
        action = self.policy.forward(obs)
        vx, vy, vz, wz = action_to_velocity(action)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
        cmd.angular.z = wz
        self.pub.publish(cmd)
        self.get_logger().info(f"发出速度: {vx:.2f}, {vy:.2f}, {vz:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PpoNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
