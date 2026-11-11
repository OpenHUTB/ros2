#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CARLA 端到端神经网络（图像 → 控制）—— ROS 2 Humble 节点。

职责：
  * 连接 CARLA、生成自车、挂载前视 RGB 相机
  * 相机图像直接送入端到端 CNN，输出转向角（**中间无人工分层的感知/规划环节**）
  * 把图像与转向指令发布到话题，便于外部可视化与记录

发布：  /carla/ego_vehicle/end_to_end/image       (sensor_msgs/Image)
        /carla/ego_vehicle/end_to_end/steer_cmd   (std_msgs/Float32)
        /carla/ego_vehicle/end_to_end/control     (std_msgs/Float32MultiArray, [throttle, steer])

启动：  ros2 launch carla_end_to_end_nn main.launch.py host:=<宿主机IP> model_path:=models/cnn.json
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray

_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_end_to_end_nn import carla_common as cc                              # noqa: E402
from carla_end_to_end_nn.main import IMG_H, IMG_W, load_cnn, cnn_predict, DT     # noqa: E402


class EndToEndNode(Node):
    """端到端驾驶节点：相机图像 → CNN → 转向。"""

    def __init__(self):
        super().__init__("carla_end_to_end_node")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2000)
        self.declare_parameter("town", "Town05")
        self.declare_parameter("model_path", "models/cnn.json")
        self.declare_parameter("backend", "numpy")
        self.declare_parameter("throttle", 0.45)
        self.declare_parameter("publish_image", True)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.town = self.get_parameter("town").value
        self.model_path = self.get_parameter("model_path").value
        self.backend = self.get_parameter("backend").value
        self.throttle = float(self.get_parameter("throttle").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)

        # ---------------- 端到端模型 ----------------
        self.net = self._load_or_train(self.model_path)

        # ---------------- 话题 ----------------
        self.pub_img = self.create_publisher(Image, "/carla/ego_vehicle/end_to_end/image", 10)
        self.pub_steer = self.create_publisher(Float32, "/carla/ego_vehicle/end_to_end/steer_cmd", 10)
        self.pub_ctrl = self.create_publisher(
            Float32MultiArray, "/carla/ego_vehicle/end_to_end/control", 10)

        # ---------------- CARLA ----------------
        self.world = None
        self.vehicle = None
        self.sensors = []
        self.img = None
        self.steer = 0.0
        self._connect()

        self.create_timer(DT, self._on_timer)
        self.get_logger().info(
            f"端到端节点已启动：host={self.host}:{self.port} town={self.town} "
            f"模型={self.model_path}（图像 {IMG_W}×{IMG_H} → CNN → 转向）")

    # ------------------------------------------------------------------ 模型
    def _load_or_train(self, model_path):
        if os.path.isfile(model_path):
            try:
                net = load_cnn(model_path, backend=self.backend)
                self.get_logger().info(f"已加载端到端 CNN：{model_path}")
                return net
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"模型加载失败({exc})，改为现场训练")
        # 纯 numpy 的 CNN 训练较慢（实测约 15 s/epoch @400 样本）。
        # 必须把预算压小并**打印进度**，否则节点静默好几分钟，使用者会以为卡死。
        self.get_logger().info(
            "未找到端到端模型，用合成道路图像现场训练（纯 numpy，120 样本 × 20 轮，约 1~2 分钟）...")
        self.get_logger().info(
            f"需要更高精度请先离线训练： python3 main.py --mode train --epochs 60 "
            f"--samples 400 --model_path {model_path}")
        from carla_end_to_end_nn.main import synth_dataset, train_cnn
        X, Y = synth_dataset(n=120, seed=0)
        net, _h = train_cnn(X, Y, model_path, epochs=20, backend=self.backend, verbose=5)
        self.get_logger().info("端到端 CNN 现场训练完成")
        return net

    # ------------------------------------------------------------------ 连接
    def _connect(self):
        try:
            _client, self.world = cc.connect(self.host, self.port, self.town)
            self.vehicle, _tf = cc.spawn_vehicle(self.world)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"连接 CARLA 失败：{exc}")
            self.get_logger().error(
                "请先启动 CARLA 服务端；启动方式与 IP/端口查看见 "
                "https://openhutb.github.io/ros2/set_up_and_connect_to_carla/")
            return
        self.sensors.append(cc.make_rgb_camera(
            self.world, self.vehicle, self._on_image,
            width=IMG_W, height=IMG_H, tick=True))
        self.get_logger().info("自车与前视相机就绪")

    def _on_image(self, img):
        self.img = img

    # ------------------------------------------------------------------ 主循环
    def _on_timer(self):
        if self.world is None or self.vehicle is None:
            return
        self.world.tick()

        img = self.img
        if img is None:
            return

        # ---- 端到端推理：图像直接 → 转向，无中间解析环节 ----
        self.steer = cnn_predict(self.net, img, backend=self.backend)
        cc.apply_control(self.vehicle, throttle=self.throttle, steer=self.steer)

        # ---- 广播 ----
        self.pub_steer.publish(Float32(data=float(self.steer)))
        self.pub_ctrl.publish(Float32MultiArray(data=[self.throttle, float(self.steer)]))
        if self.publish_image:
            self.pub_img.publish(self._to_image_msg(img))

    def _to_image_msg(self, img):
        """numpy HWC uint8 → sensor_msgs/Image（编码 rgb8）。"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ego_vehicle"
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = int(img.shape[1] * 3)
        msg.data = np.ascontiguousarray(img[:, :, :3], dtype=np.uint8).tobytes()
        return msg

    def destroy_node(self):
        for s in self.sensors:
            try:
                s.stop()
                s.destroy()
            except Exception:  # noqa: BLE001
                pass
        if self.vehicle is not None:
            try:
                self.vehicle.destroy()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EndToEndNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
