"""sim_node.py —— 多模态仿真节点（发布图像+IMU）"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from humanoid_fall_warning.sim import FallWarningSim, DEFAULT_STAND_TARGET

def _model():
    try:
        share = get_package_share_directory("humanoid_fall_warning")
        p = os.path.join(share, "models", "humanoid_warning.xml")
        if os.path.isfile(p): return p
    except: pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "models", "humanoid_warning.xml")

class SimNode(Node):
    def __init__(self):
        super().__init__("warning_sim_node")
        self.sim = FallWarningSim(_model()); self.sim.reset()
        self.pub_img = self.create_publisher(Image, "front_image", 10)
        self.pub_imu = self.create_publisher(Float64MultiArray, "imu_torso", 10)
        self.sub = self.create_subscription(Float64MultiArray, "joint_target", self._cb, 10)
        self.target = DEFAULT_STAND_TARGET.copy()
        self.timer = self.create_timer(0.01, self._tick)
    def _cb(self, msg):
        if len(msg.data) == 15: self.target = np.array(msg.data)
    def _tick(self):
        for _ in range(5): self.sim.step(target=self.target)
        img = self.sim.render_front(gray=True)
        m = Image(); m.height, m.width = img.shape; m.encoding = "mono8"
        m.data = (img*255).astype(np.uint8).tobytes()
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub_img.publish(m)
        pitch, roll = self.sim.torso_pose()
        imu = Float64MultiArray(); imu.data = [pitch, roll]
        self.pub_imu.publish(imu)

def main():
    rclpy.init(); n = SimNode()
    try: rclpy.spin(n)
    except: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__ == "__main__": main()
