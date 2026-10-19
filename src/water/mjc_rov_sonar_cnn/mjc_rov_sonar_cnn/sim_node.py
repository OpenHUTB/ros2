"""
sim_node.py — ROS 仿真节点：加载 MuJoCo 模型，发布合成声呐图像与 ROV odom。
"""
import os
os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from . import common


class SimNode(Node):
    def __init__(self):
        super().__init__("rov_sonar_sim")
        self.declare_parameter("odom_rate", 20.0)
        self.declare_parameter("image_rate", 5.0)
        self.declare_parameter("sonar_width", 64)
        self.declare_parameter("sonar_height", 64)

        import mujoco
        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(common.find_model_path())
        self.data = mujoco.MjData(self.model)

        qos = QoSProfile(depth=2)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pub_img = self.create_publisher(Image, "/rov_sonar/image_raw", qos)
        self.pub_odom = self.create_publisher(Odometry, "/rov_sonar/odom", 10)

        self.create_timer(1.0/self.get_parameter("odom_rate").value, self._pub_odom)
        self.create_timer(1.0/self.get_parameter("image_rate").value, self._pub_image)
        self.get_logger().info("rov_sonar_sim 已启动")

    def _pub_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.data.qpos[0])
        msg.pose.pose.position.y = float(self.data.qpos[1])
        msg.pose.pose.position.z = float(self.data.qpos[2])
        self.pub_odom.publish(msg)

    def _pub_image(self):
        try:
            rov_xy = np.array([self.data.qpos[0], self.data.qpos[1]])
            img = common.generate_sonar_image(
                rov_xy,
                rock_pos=[[1.5,1.5],[-2.0,1.0],[-1.0,-2.0]],
                pipe_pos=[[2.5,-1.5],[-2.5,-1.0]],
                wreck_pos=[[0.5,-2.5],[-0.5,2.5]],
                size=64,
            )
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = img.shape
            msg.encoding = "mono8"
            msg.step = msg.width
            msg.data = img.tobytes()
            self.pub_img.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"声呐图渲染失败: {e}")


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


if __name__ == "__main__":
    main()
