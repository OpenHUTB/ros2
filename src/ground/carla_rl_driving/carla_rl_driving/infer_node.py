# -*- coding: utf-8 -*-
"""
部署推理节点：infer_node（任务四：端到端模型在线部署）

rclpy (ROS2 Humble) 节点：
    订阅相机图像  ->  预处理  ->  加载训练好的策略网络  ->  发布阿克曼控制指令
                                                              (20 Hz)

话题全部可在 config/rl_driving.yaml 里改，默认对接 CARLA ros-bridge：
    订阅 : /carla/ego_vehicle/rgb_view/image        (sensor_msgs/Image)
    发布 : /carla/ego_vehicle/ackermann_cmd          (carla_msgs/CarlaAckermannControl)

启动：
    ros2 launch carla_rl_driving carla_rl_infer.launch.py
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from carla_msgs.msg import CarlaAckermannControl
except Exception:  # pragma: no cover
    CarlaAckermannControl = None

from .utils import preprocess_image


class InferNode(Node):
    def __init__(self):
        super().__init__('carla_rl_infer_node')

        # ---- 参数（launch/yaml 可覆盖）----
        self.declare_parameter('image_topic', '/carla/ego_vehicle/rgb_view/image')
        self.declare_parameter('control_topic', '/carla/ego_vehicle/ackermann_cmd')
        self.declare_parameter('model_path', 'models/ppo_cnn_town01.zip')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('ctrl_rate', 20.0)

        image_topic = self.get_parameter('image_topic').value
        control_topic = self.get_parameter('control_topic').value
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        rate = float(self.get_parameter('ctrl_rate').value)

        # ---- 加载训练好的策略（SB3 保存的 zip）----
        self.model = None
        if os.path.exists(model_path):
            try:
                from stable_baselines3 import PPO
                self.model = PPO.load(model_path, device=device)
                self.get_logger().info('已加载策略模型：%s' % model_path)
            except Exception as e:
                self.get_logger().warn('加载 SB3 模型失败（%s），将输出零控制' % e)
        else:
            self.get_logger().warn('模型文件不存在：%s，节点空转等待' % model_path)

        self._last_action = np.zeros(2, dtype=np.float32)

        # ---- 订阅 / 发布 ----
        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, 10)
        if CarlaAckermannControl is not None:
            self.pub = self.create_publisher(CarlaAckermannControl, control_topic, 10)
        else:
            self.pub = None
            self.get_logger().warn('未找到 carla_msgs，控制指令将只打印日志')

        self.timer = self.create_timer(1.0 / rate, self._step)
        self._img = None

    def _on_image(self, msg):
        h, w = msg.height, msg.width
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        self._img = arr

    def _step(self):
        if self._img is None or self.model is None:
            return
        obs = preprocess_image(self._img, target_h=80, target_w=160)
        obs = np.expand_dims(obs, 0)                 # (1,3,H,W)
        action, _ = self.model.predict(obs, deterministic=True)
        steer, accel = float(action[0]), float(action[1])
        self._last_action = np.array([steer, accel], dtype=np.float32)

        if self.pub is not None:
            cmd = CarlaAckermannControl()
            cmd.steer = steer
            cmd.speed = max(accel, 0.0) * 10.0       # 简化映射，完整见 ackermann 控制包
            self.pub.publish(cmd)
        self.get_logger().debug('steer=%.2f accel=%.2f' % (steer, accel))


def main(args=None):
    rclpy.init(args=args)
    node = InferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
