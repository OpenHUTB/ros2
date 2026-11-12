"""ROS2 节点：订阅 IMU 时序，发布姿态角估计。"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from .mlp import MLP

WINDOW = 5


class AttitudeNode(Node):
    def __init__(self):
        super().__init__("uav_airsim_attitude_estimator")
        self.declare_parameter("weights", "models_trained/attitude.npz")
        wpath = self.get_parameter("weights").value
        if not os.path.isabs(wpath) and not os.path.exists(wpath):
            # 相对路径且当前工作目录找不到时，按本包根目录（models_trained 所在那一层）解析
            wpath = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), wpath
            )
        if not os.path.exists(wpath):
            raise FileNotFoundError(
                "权重文件不存在：%s（用 weights 参数传绝对路径，或先运行 python3 main.py --train 生成）"
                % wpath
            )

        w = np.load(wpath)
        self.net = MLP(n_in=30, hidden=(32,), n_out=2, seed=0)
        self.net.load_state({k: w[k] for k in ("W0", "b0", "W1", "b1")})
        self.x_mean, self.x_std = w["x_mean"], w["x_std"]
        self.y_mean, self.y_std = w["y_mean"], w["y_std"]

        self.buf = []
        self.pub = self.create_publisher(Float32MultiArray, "/uav/attitude", 10)
        self.sub = self.create_subscription(Imu, "/uav/imu", self.on_imu, 10)
        self.get_logger().info("姿态估计节点就绪，权重 %s" % wpath)

    def on_imu(self, msg: Imu):
        vec = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        ], dtype=np.float32)
        self.buf.append(vec)
        if len(self.buf) > WINDOW:
            self.buf.pop(0)
        while len(self.buf) < WINDOW:
            self.buf.insert(0, vec)

        x = np.concatenate(self.buf, axis=0)[None, :]
        pred = self.net.forward(((x - self.x_mean) / self.x_std).astype(np.float32))
        out = (pred * self.y_std + self.y_mean).ravel()
        m = Float32MultiArray()
        m.data = [float(out[0]), float(out[1])]
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
