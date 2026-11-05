#!/usr/bin/env python3
"""uav_vision_nn 总入口：推理节点。

订阅相机图像话题，用训练好的 CNN 回归机体系速度，发布 Twist 速度指令。
入口名 `main`，因此 `ros2 run uav_vision_nn main` 与 launch 里的
`Node(executable='main')` 都指向本文件的 main()。
"""

import os
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from uav_vision_nn.model import DrivingCNN, preprocess_image


def image_msg_to_numpy(msg):
    """sensor_msgs/Image -> np.ndarray，支持 bgr8 / rgb8 / mono8。"""
    if msg.encoding == 'mono8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

    if msg.encoding in ('bgr8', 'rgb8'):
        array = np.frombuffer(msg.data, dtype=np.uint8)
        array = array.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            array = np.ascontiguousarray(array[:, :, ::-1])
        return array

    raise ValueError(f'不支持的图像编码: {msg.encoding}')


class UavVisionNode(Node):
    """图像 -> 神经网络 -> 速度指令。"""

    def __init__(self):
        super().__init__('uav_vision_node')

        self.declare_parameter('model_path', '~/.ros/uav_vision_nn/model.pth')
        self.declare_parameter('image_size', 64)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('device', 'cpu')

        model_path = os.path.expanduser(
            str(self.get_parameter('model_path').value))
        self._image_size = int(self.get_parameter('image_size').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._device = str(self.get_parameter('device').value)
        rate = max(float(self.get_parameter('publish_rate').value), 0.1)
        self._min_interval = 1.0 / rate

        try:
            self._model = DrivingCNN.load(
                model_path, image_size=self._image_size, device=self._device)
        except (FileNotFoundError, RuntimeError) as exc:
            self.get_logger().fatal(f'加载模型失败: {exc}')
            raise

        self.get_logger().info(
            f'模型已加载: {model_path} | 输入 {self._image_size}x{self._image_size} '
            f'| 参数 {self._model.num_parameters()} | 设备 {self._device}')

        self._pub_cmd = self.create_publisher(Twist, '~/cmd_vel', 10)
        self._pub_infer = self.create_publisher(Float32, '~/infer_ms', 10)
        # 队列深度 1：控制回路只要最新帧，旧帧直接丢弃
        self.create_subscription(Image, '~/image', self._on_image, 1)

        self._last_time = 0.0
        self.get_logger().info(
            f'推理节点就绪，限速 {self._max_speed} m/s，频率 {rate:.1f} Hz')

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self._last_time < self._min_interval:
            return
        self._last_time = now

        try:
            frame = image_msg_to_numpy(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        begin = time.perf_counter()
        try:
            tensor = torch.from_numpy(preprocess_image(frame, self._image_size))
            with torch.no_grad():
                raw = self._model(tensor.to(self._device))[0].cpu().numpy()
        except Exception as exc:  # 单帧异常不应拖垮整个控制回路
            self.get_logger().warn(f'推理失败: {exc}', throttle_duration_sec=5.0)
            return
        infer_ms = (time.perf_counter() - begin) * 1000.0

        vx = float(np.clip(raw[0], -self._max_speed, self._max_speed))
        vy = float(np.clip(raw[1], -self._max_speed, self._max_speed))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        self._pub_cmd.publish(cmd)
        self._pub_infer.publish(Float32(data=float(infer_ms)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UavVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # 启动期失败给出可读原因，避免 launch 只打印堆栈
        print(f'[uav_vision_nn] 推理节点启动失败: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
