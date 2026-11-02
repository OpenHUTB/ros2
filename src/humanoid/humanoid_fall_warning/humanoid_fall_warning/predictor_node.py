"""predictor_node.py —— 多模态跌倒预警推理节点"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from humanoid_fall_warning.predictor import FallWarningPredictor
from humanoid_fall_warning.sim import DEFAULT_STAND_TARGET

def _weights():
    try:
        share = get_package_share_directory("humanoid_fall_warning")
        p = os.path.join(share, "models_trained", "fall_warning.npz")
        if os.path.isfile(p): return p
    except: pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "models_trained", "fall_warning.npz")

class PredictorNode(Node):
    def __init__(self):
        super().__init__("warning_predictor_node")
        self.pred = FallWarningPredictor(_weights())
        self._pitch = 0.0; self._roll = 0.0
        self.pub = self.create_publisher(Float64MultiArray, "joint_target", 10)
        self.sub_img = self.create_subscription(Image, "front_image", self._img_cb, 10)
        self.sub_imu = self.create_subscription(Float64MultiArray, "imu_torso", self._imu_cb, 10)
        self._warn_count = 0
    def _imu_cb(self, msg):
        if len(msg.data) == 2:
            self._pitch, self._roll = float(msg.data[0]), float(msg.data[1])
    def _img_cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width).astype(np.float32)/255.0
        size = 16; bh, bw = img.shape[0]//size, img.shape[1]//size
        crop = img[:bh*size, :bw*size]
        feat = crop.reshape(size, bh, size, bw).mean(axis=(1,3)).flatten()
        p_normal, p_fall = self.pred.predict(feat, self._pitch, self._roll)
        target = DEFAULT_STAND_TARGET.copy()
        if p_fall > 0.6:
            # 即将跌倒：紧急下蹲姿态
            target[1] = 0.3; target[6] = 0.3
            target[2] = -0.4; target[7] = -0.4
            self._warn_count += 1
            if self._warn_count % 30 == 0:
                self.get_logger().warn(f"FALL WARNING p_fall={p_fall:.2f}, triggering squat")
        out = Float64MultiArray(); out.data = target.tolist()
        self.pub.publish(out)

def main():
    rclpy.init(); n = PredictorNode()
    try: rclpy.spin(n)
    except: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
