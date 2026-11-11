"""ROS 2 节点：订阅 IMU 与位置，发布抗风补偿速度。"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

from .mlp import MLP

WINDOW = 5


class WindRejectNode(Node):
    def __init__(self):
        super().__init__("uav_wind_rejection")

        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_w = os.path.join(root, "models_trained", "wind.npz")
        default_d = os.path.join(root, "models_trained", "wind_data.npz")
        self.declare_parameter("weights", default_w)
        self.declare_parameter("data", default_d)
        self.declare_parameter("cmd_scale", 3.0)
        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.0)

        self.scale = float(self.get_parameter("cmd_scale").value)
        self.tx = float(self.get_parameter("target_x").value)
        self.ty = float(self.get_parameter("target_y").value)

        self.net = MLP(n_in=32, hidden=(32,), n_out=2)
        self.net.load(self.get_parameter("weights").value)
        d = np.load(self.get_parameter("data").value)
        self.mean, self.std = d["x_mean"], d["x_std"]

        self.buf = []
        self.pos = None

        self.pub = self.create_publisher(Float32MultiArray, "/uav/wind_cmd", 10)
        self.create_subscription(Imu, "/uav/imu", self.on_imu, 10)
        self.create_subscription(PoseStamped, "/uav/pose", self.on_pose, 10)
        self.get_logger().info("抗风补偿节点就绪")

    def on_pose(self, msg: PoseStamped):
        self.pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def on_imu(self, msg: Imu):
        a, g = msg.linear_acceleration, msg.angular_velocity
        vec = np.array([a.x, a.y, a.z, g.x, g.y, g.z], dtype=np.float32)
        self.buf.append(vec)
        if len(self.buf) > WINDOW:
            self.buf.pop(0)
        while len(self.buf) < WINDOW:
            self.buf.insert(0, vec)
        if self.pos is None:
            return

        err = np.array([self.tx, self.ty]) - self.pos
        x = np.concatenate([np.concatenate(self.buf), err.astype(np.float32)])
        x = ((x - self.mean) / self.std).astype(np.float32)
        act = self.net.forward(x) * self.scale

        out = Float32MultiArray()
        out.data = [float(act[0]), float(act[1])]
        self.pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = WindRejectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
