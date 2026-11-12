"""dock_node.py — CNN 检测对接标记 + 逼近导航。"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from . import common
from .dock_cnn import DockCNN
import torch


class DockNode(Node):
    def __init__(self):
        super().__init__("rov_dock_node")
        wpath = common.find_weights_path(self.declare_parameter("weights_path", "").value)
        if not wpath.exists():
            raise FileNotFoundError(f"找不到 CNN 权重 {wpath}，请先运行: python3 main.py train")
        self.net = DockCNN()
        self.net.load_state_dict(torch.load(wpath, map_location="cpu"))
        self.net.eval()
        self.create_subscription(Float32MultiArray, "/rov_dock/sonar_scan", self._on_scan, 10)
        self.pub = self.create_publisher(Twist, "/rov_dock/cmd_vel", 10)
        self.get_logger().info("rov_dock_node 已启动")

    def _on_scan(self, msg: Float32MultiArray):
        scan = torch.tensor(msg.data, dtype=torch.float32).unsqueeze(0).unsqueeze(0)
        with torch.no_grad():
            out = self.net(scan)
            has_dock = out.argmax(dim=1).item()
        cmd = Twist()
        if has_dock:
            cmd.linear.x = 0.5  # 检测到对接站，前进
            self.get_logger().info_throttle(2.0, "检测到对接标记，逼近中")
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = DockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
