#!/usr/bin/env python3
"""人形机器人桥接节点：将cmd_vel通过Gazebo服务作用到模型"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist as GzTwist

class HumanoidBridge(Node):
    def __init__(self):
        super().__init__('humanoid_bridge')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.sub = self.create_subscription(Twist, '/humanoid/cmd_vel', self.cb, 10)
        self.model_name = self.declare_parameter('model_name', 'humanoid').value

    def cb(self, msg):
        if not self.client.wait_for_service(timeout_sec=0.1): return
        req = SetEntityState.Request()
        req.state.model_name = self.model_name
        req.state.pose = Pose()
        req.state.twist = GzTwist()
        req.state.twist.linear.x = msg.linear.x
        req.state.twist.linear.y = msg.linear.y
        req.state.twist.angular.z = msg.angular.z
        self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
