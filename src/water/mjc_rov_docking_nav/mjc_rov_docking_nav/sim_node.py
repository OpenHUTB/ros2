"""sim_node.py — MuJoCo 仿真 ROS 节点，发布 odom 和声呐扫描。"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from . import common


class SimNode(Node):
    def __init__(self):
        super().__init__("rov_dock_sim")
        self.declare_parameter("odom_rate", 20.0)
        import mujoco
        self.model = mujoco.MjModel.from_xml_path(str(common.get_package_root() / "models" / "rov_docking_pool.xml"))
        self.data = mujoco.MjData(self.model)
        self.pub_odom = self.create_publisher(Odometry, "/rov_dock/odom", 10)
        self.pub_scan = self.create_publisher(Float32MultiArray, "/rov_dock/sonar_scan", 10)
        self.create_timer(1.0/self.get_parameter("odom_rate").value, self._pub)
        self.get_logger().info("rov_dock_sim 已启动")

    def _pub(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.data.qpos[0])
        msg.pose.pose.position.y = float(self.data.qpos[1])
        self.pub_odom.publish(msg)
        scan = common.sonar_scan(
            np.array([self.data.qpos[0], self.data.qpos[1]]),
            np.array([3.0, 0.0]),
            [[1.5, 1.5], [1.5, -1.5]],
        )
        s = Float32MultiArray()
        s.data = scan.tolist()
        self.pub_scan.publish(s)


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
