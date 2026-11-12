"""predictor_node.py —— 多模态跌倒预警推理节点"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from humanoid_fall_warning.dataset import feat_from_gray
from humanoid_fall_warning.predictor import FallWarningPredictor
from humanoid_fall_warning.sim import DEFAULT_STAND_TARGET, SQUAT_TARGET

WARN_THRESHOLD = 0.6


def _weights():
    """优先取 install/share 下的权重，否则回落到源码树 package 根下的 models_trained。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("humanoid_fall_warning")
        p = os.path.join(share, "models_trained", "fall_warning.npz")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "models_trained", "fall_warning.npz"))


class PredictorNode(Node):
    def __init__(self):
        super().__init__("warning_predictor_node")
        self.pred = FallWarningPredictor(_weights())
        self._pitch = 0.0
        self._roll = 0.0
        self.pub = self.create_publisher(Float64MultiArray, "joint_target", 10)
        self.sub_img = self.create_subscription(Image, "front_image", self._img_cb, 10)
        self.sub_imu = self.create_subscription(Float64MultiArray, "imu_torso", self._imu_cb, 10)
        self._warn_count = 0
        self._last_warn = False

    def _imu_cb(self, msg):
        if len(msg.data) == 2:
            self._pitch, self._roll = float(msg.data[0]), float(msg.data[1])

    def _img_cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        feat = feat_from_gray(img.astype(np.float32) / 255.0)
        p_normal, p_fall = self.pred.predict(feat, self._pitch, self._roll)
        target = DEFAULT_STAND_TARGET.copy()
        warn = p_fall > WARN_THRESHOLD
        if warn:
            # 紧急下蹲：髋屈 + 膝屈 + 踝背屈。踝背屈把重心拉回足底支撑面内，缺了会前滑。
            target = SQUAT_TARGET.copy()
            self._warn_count += 1
            if self._warn_count % 30 == 0:
                self.get_logger().warn(f"FALL WARNING p_fall={p_fall:.2f}, triggering squat")
        elif self._last_warn:
            self.get_logger().info("预警解除，回到站姿")
        self._last_warn = warn
        out = Float64MultiArray()
        out.data = target.tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = PredictorNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
