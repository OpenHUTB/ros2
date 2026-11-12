"""ROS 2 节点：订阅 IMU，发布神经网络估计的速度大小。"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from .mlp import MLP

WINDOW = 5


class ImuVelNode(Node):
    def __init__(self):
        super().__init__("uav_imu_velocity_estimator")

        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_w = os.path.join(pkg_root, "models_trained", "imu_vel.npz")
        self.declare_parameter("weights", default_w)
        self.declare_parameter("data", os.path.join(pkg_root, "models_trained",
                                                    "imu_vel_data.npz"))
        self.declare_parameter("vel_scale", 4.0)

        wpath = self.get_parameter("weights").value
        dpath = self.get_parameter("data").value
        self.scale = float(self.get_parameter("vel_scale").value)

        self.net = MLP(n_in=6 * WINDOW, hidden=(32,), n_out=1)
        self.net.load(wpath)

        d = np.load(dpath)
        self.mean = d["x_mean"]
        self.std = d["x_std"]

        self.buf = []
        self.pub = self.create_publisher(Float32, "/uav/velocity_est", 10)
        self.sub = self.create_subscription(Imu, "/uav/imu", self.on_imu, 10)
        self.get_logger().info("已加载权重 %s" % wpath)

    def on_imu(self, msg: Imu):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        vec = np.array([a.x, a.y, a.z, g.x, g.y, g.z], dtype=np.float32)
        self.buf.append(vec)
        if len(self.buf) > WINDOW:
            self.buf.pop(0)
        while len(self.buf) < WINDOW:
            self.buf.insert(0, vec)

        x = (np.concatenate(self.buf, axis=0) - self.mean) / self.std
        v = float(self.net.forward(x.astype(np.float32))[0]) * self.scale

        out = Float32()
        out.data = v
        self.pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = ImuVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
