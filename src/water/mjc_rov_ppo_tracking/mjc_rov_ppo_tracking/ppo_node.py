"""ppo_node.py — PPO 推理节点，发布推力到仿真。"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import torch
from . import common
from .ppo_agent import Actor


class PpoNode(Node):
    def __init__(self):
        super().__init__("rov_ppo_node")
        wpath = common.find_weights_path(self.declare_parameter("weights_path", "").value)
        if not wpath.exists():
            raise FileNotFoundError(f"找不到 PPO 权重 {wpath}，请先运行: python3 main.py train")
        self.actor = Actor()
        self.actor.load_state_dict(torch.load(wpath, map_location="cpu"))
        self.actor.eval()
        self.get_logger().info(f"PPO actor 已加载: {wpath}")
        self.create_subscription(Odometry, "/rov_ppo/odom", self._on_odom, 10)
        self.pub = self.create_publisher(Twist, "/rov_ppo/cmd_vel", 10)

    def _on_odom(self, msg: Odometry):
        s = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0, 0.0])
        with torch.no_grad():
            mu, _ = self.actor(torch.tensor(s, dtype=torch.float32))
            a = mu.numpy()
        cmd = Twist()
        cmd.linear.x = float(np.clip(a[0], -5, 5))
        cmd.linear.y = float(np.clip(a[1], -5, 5))
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PpoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
