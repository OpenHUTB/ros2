"""Perception node: /scan (LaserScan) -> CNN inference -> /obstacle_info (Float32MultiArray).

Output layout: [nearest_dist, front_clear, left_clear, right_clear] in meters.
"""
from __future__ import annotations
import os
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

MAX_RANGE = 8.0


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        share = get_package_share_directory("humanoid_lidar_nn_perception")
        cnn_path = os.path.join(share, "weights", "perception_cnn.pt")
        self.model = torch.jit.load(cnn_path)
        self.model.eval()

        self.pub = self.create_publisher(Float32MultiArray, "/obstacle_info", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.get_logger().info(f"PerceptionNode ready. cnn={cnn_path}")

    def _scan_cb(self, msg: LaserScan):
        scan = np.array(msg.ranges, dtype=np.float32)
        scan = np.nan_to_num(scan, nan=MAX_RANGE, posinf=MAX_RANGE, neginf=0.1)
        with torch.no_grad():
            pred = self.model(torch.from_numpy(scan).unsqueeze(0))[0].numpy()
        out = Float32MultiArray()
        out.data = [
            float(pred[0] * MAX_RANGE),  # nearest
            float(pred[1] * MAX_RANGE),  # front
            float(pred[2] * MAX_RANGE),  # left
            float(pred[3] * MAX_RANGE),  # right
        ]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
