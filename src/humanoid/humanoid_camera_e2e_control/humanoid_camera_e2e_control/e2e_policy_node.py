# -*- coding: utf-8 -*-
"""
e2e_policy_node.py —— 图像端到端策略 ROS2 节点。

订阅仿真节点发布的跟随视角相机图像，加载训练好的 E2ECNN，直接输出速度指令并发布
到 /cmd_vel，形成"图像 → CNN → 控制指令"的闭环。本节点不包含显式感知模块
或规则避障：决策完全由卷积神经网络从像素回归得到。
"""

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

try:
    from humanoid_camera_e2e_control.network import E2ECNN
    from humanoid_camera_e2e_control.data_io import preprocess
    from humanoid_camera_e2e_control.sim_core import calibrate_command
except ImportError:
    from network import E2ECNN
    from data_io import preprocess
    from sim_core import calibrate_command


class E2EPolicyNode(Node):
    """跟随视角图像 → CNN → /cmd_vel 的端到端策略节点。"""

    def __init__(self):
        super().__init__('e2e_policy_node')

        self.declare_parameter('weights_path', '')
        self.declare_parameter('image_topic', '/humanoid/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('device', 'cpu')

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        period = self.get_parameter('control_period').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value

        if not weights_path:
            self.get_logger().error('未配置 weights_path，请先训练或指定权重文件。')
            raise RuntimeError('weights_path is required')

        import torch
        self.torch = torch
        self.device = device if (device == 'cuda' and torch.cuda.is_available()) else 'cpu'
        self.net = E2ECNN.load(weights_path)
        self.net.to(self.device)
        self.net.eval()

        self._latest = None
        self._lock = threading.Lock()

        self.sub_image = self.create_subscription(Image, image_topic, self._image_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(period, self._infer)

        self.get_logger().info(
            f'端到端策略节点已启动 | 权重 {weights_path} | 设备 {self.device} | '
            f'图像 {image_topic} -> 指令 {cmd_topic} | 周期 {period*1000:.0f} ms')

    def _image_cb(self, msg: Image):
        if msg.encoding != 'rgb8':
            self.get_logger().warning(f'非 rgb8 图像（{msg.encoding}），已跳过。')
            return
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, 3)
        with self._lock:
            self._latest = img

    def _infer(self):
        with self._lock:
            if self._latest is None:
                return
            img = self._latest.copy()

        x = preprocess(img).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            v, w = calibrate_command(float(self.net(x)[0][0]), float(self.net(x)[0][1]))
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = E2EPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
