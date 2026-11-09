"""sim_node.py — MuJoCo 仿真 ROS 节点。"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from . import common

class SimNode(Node):
    def __init__(self):
        super().__init__("rov_rl_sim")
        import mujoco
        self.model = mujoco.MjModel.from_xml_path(str(common.get_package_root() / "models" / "rov_rl_pool.xml"))
        self.data = mujoco.MjData(self.model)
        self.pub = self.create_publisher(Odometry, "/rov_rl/odom", 10)
        self.create_timer(0.05, self._pub)
        self.get_logger().info("rov_rl_sim 已启动")
    def _pub(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.data.qpos[0])
        msg.pose.pose.position.y = float(self.data.qpos[1])
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SimNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()
