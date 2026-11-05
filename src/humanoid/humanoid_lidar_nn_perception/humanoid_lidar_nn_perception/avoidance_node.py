"""Avoidance node: NN controller (behavior-cloned MLP).

Input  : [nearest, front, left, right, cos(target_wz), sin(target_wz)] from /obstacle_info + /cmd_vel
Output : (vx, wz) safe command -> /cmd_vel_safe

The MLP was trained by behavior cloning from the reactive rule; it is a real
neural network (32-32 hidden), not an if-else chain.
"""
from __future__ import annotations
import os
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__("avoidance_node")
        self.obstacle = np.zeros(4, dtype=np.float32)
        self.user_cmd = np.zeros(2, dtype=np.float32)

        # load NN controller
        weights_dir = os.path.join(get_package_share_directory("humanoid_lidar_nn_perception"), "weights")
        mlp_path = os.path.join(weights_dir, "avoid_mlp.pt")
        self.mlp = torch.jit.load(mlp_path, map_location="cpu")
        self.mlp.eval()

        self.create_subscription(Float32MultiArray, "/obstacle_info", self._obs_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.timer = self.create_timer(0.1, self._control)
        self.get_logger().info(f"AvoidanceNode ready (NN controller: {mlp_path}).")

    def _obs_cb(self, msg: Float32MultiArray):
        self.obstacle = np.array(msg.data, dtype=np.float32)

    def _cmd_cb(self, msg: Twist):
        self.user_cmd[0] = msg.linear.x
        self.user_cmd[1] = msg.angular.z

    def _control(self):
        # normalize target_wz to [-1,1] -> cos/sin embedding
        target_wz = float(np.clip(self.user_cmd[1], -1.0, 1.0))
        obs = np.array([
            self.obstacle[0],  # nearest / 8
            self.obstacle[1],  # front  / 8
            self.obstacle[2],  # left   / 8
            self.obstacle[3],  # right  / 8
            np.cos(target_wz),
            np.sin(target_wz),
        ], dtype=np.float32)

        with torch.no_grad():
            act = self.mlp(torch.from_numpy(obs).unsqueeze(0))[0].numpy()
        # act in [-1,1] -> vx in [0, 0.4], wz in [-0.8, 0.8]
        vx = float(np.clip(act[0], -1, 1)) * 0.225 + 0.175
        wz = float(np.clip(act[1], -1, 1)) * 0.8

        out = Twist()
        out.linear.x = vx
        out.angular.z = wz
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
