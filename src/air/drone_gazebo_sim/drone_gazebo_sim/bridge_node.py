#!/usr/bin/env python3
"""无人机物理桥接节点
订阅 /drone/cmd_vel，通过 gazebo 服务设置模型位姿实现运动
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math


class DroneBridge(Node):
    def __init__(self):
        super().__init__('drone_bridge')
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('等待gazebo服务...')
        self.sub = self.create_subscription(Twist, '/drone/cmd_vel', self.cmd_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0  # 起飞高度
        self.yaw = 0.0
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info('无人机物理桥接节点已启动')

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.vyaw = msg.angular.z

    def step(self):
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt
        self.yaw += self.vyaw * self.dt
        self.z += self.vz * self.dt
        self.z = max(0.2, min(5.0, self.z))  # 高度限制
        req = SetEntityState.Request()
        state = EntityState()
        state.name = 'simple_quad'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.z = math.sin(self.yaw / 2)
        state.pose.orientation.w = math.cos(self.yaw / 2)
        req.state = state
        self.cli.call_async(req)


def main():
    rclpy.init()
    node = DroneBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
