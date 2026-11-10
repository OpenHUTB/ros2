#!/usr/bin/env python3
"""水下ROV桥接节点：将cmd_vel通过Gazebo服务作用到模型"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist as GzTwist


class RovBridge(Node):
    def __init__(self):
        super().__init__('rov_bridge')
        self.client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.sub = self.create_subscription(
            Twist, '/rov/cmd_vel', self.cb, 10)
        self.model_name = self.declare_parameter('model_name', 'rov').value
        self.get_logger().info('ROV桥接节点已启动')

    def cb(self, msg):
        if not self.client.wait_for_service(timeout_sec=0.1):
            return
        req = SetEntityState.Request()
        req.state.model_name = self.model_name
        req.state.pose = Pose()
        req.state.twist = GzTwist()
        req.state.twist.linear.x = msg.linear.x
        req.state.twist.linear.y = msg.linear.y
        req.state.twist.linear.z = msg.linear.z
        req.state.twist.angular.z = msg.angular.z
        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = RovBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
