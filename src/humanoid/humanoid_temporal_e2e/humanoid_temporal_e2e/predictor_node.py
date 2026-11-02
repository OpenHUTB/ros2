"""predictor_node.py —— ROS2推理节点（3帧时序→关节角）"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from humanoid_temporal_e2e.predictor import TemporalPredictor

def _weights():
    try:
        share = get_package_share_directory("humanoid_temporal_e2e")
        p = os.path.join(share, "models_trained", "temporal_e2e.npz")
        if os.path.isfile(p): return p
    except: pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "models_trained", "temporal_e2e.npz")

class PredictorNode(Node):
    def __init__(self):
        super().__init__("temporal_predictor_node")
        self.pred = TemporalPredictor(_weights())
        self.pub = self.create_publisher(Float64MultiArray, "joint_target", 10)
        self.sub = self.create_subscription(Image, "front_image", self._cb, 10)
    def _cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width).astype(np.float32)/255.0
        self.pred.push_frame(img)
        target = self.pred.predict()
        out = Float64MultiArray(); out.data = target.tolist()
        self.pub.publish(out)

def main():
    rclpy.init(); n = PredictorNode()
    try: rclpy.spin(n)
    except: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
