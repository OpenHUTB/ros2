"""sim_node.py —— ROS2仿真节点（深度图发布）"""
from __future__ import annotations
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
from humanoid_depth_pose_control.sim import DepthPoseSim, DEFAULT_STAND_TARGET

STEPS_PER_TICK = 5

def _model():
    try:
        share = get_package_share_directory("humanoid_depth_pose_control")
        p = os.path.join(share, "models", "humanoid_depth.xml")
        if os.path.isfile(p): return p
    except Exception: pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "..", "models", "humanoid_depth.xml")


class SimNode(Node):
    def __init__(self):
        super().__init__("depth_sim_node")
        self.sim = DepthPoseSim(_model())
        self.sim.reset()
        self.pub = self.create_publisher(Image, "depth_image", 10)
        self.pub_j = self.create_publisher(Float64MultiArray, "joint_state", 10)
        self.sub = self.create_subscription(Float64MultiArray, "joint_target", self._cb, 10)
        self.target = DEFAULT_STAND_TARGET.copy()
        self.timer = self.create_timer(0.01, self._tick)
        self.get_logger().info(f"depth_sim_node started, model={_model()}")

    def _cb(self, msg):
        if len(msg.data) == 15:
            self.target = np.array(msg.data)

    def _tick(self):
        for _ in range(STEPS_PER_TICK):
            self.sim.step(target=self.target)
        d = self.sim.render_depth()
        m = Image()
        m.height, m.width = d.shape
        m.encoding = "32FC1"
        m.data = d.astype(np.float32).tobytes()
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(m)


def main():
    rclpy.init()
    n = SimNode()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
