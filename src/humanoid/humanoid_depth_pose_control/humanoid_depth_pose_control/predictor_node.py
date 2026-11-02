"""predictor_node.py —— ROS2推理节点（深度图→姿态→关节目标）"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from humanoid_depth_pose_control.predictor import DepthPosePredictor
from humanoid_depth_pose_control.controller import ProportionalController

def _weights():
    try:
        share = get_package_share_directory("humanoid_depth_pose_control")
        p = os.path.join(share, "models_trained", "depth_pose_mlp.npz")
        if os.path.isfile(p): return p
    except Exception: pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models_trained", "depth_pose_mlp.npz")


class PredictorNode(Node):
    def __init__(self):
        super().__init__("depth_predictor_node")
        self.pred = DepthPosePredictor(_weights())
        self.ctrl = ProportionalController()
        self.pub = self.create_publisher(Float64MultiArray, "joint_target", 10)
        self.sub = self.create_subscription(Image, "depth_image", self._cb, 10)
        self.get_logger().info(f"depth_predictor_node started, weights={_weights()}")

    def _cb(self, msg):
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        p, r, y = self.pred.predict_from_depth(d)
        target = self.ctrl.target_for(p, r)
        out = Float64MultiArray()
        out.data = target.tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = PredictorNode()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
