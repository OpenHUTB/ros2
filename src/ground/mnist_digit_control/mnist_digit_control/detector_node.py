#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数字检测节点：相机帧 -> 预处理 -> CNN 推理 -> 发布数字指令。

话题：
    发布：~/digit        std_msgs/Int32     当前帧识别的数字（-1 表示未检测到）
          ~/confidence   std_msgs/Float32   对应的 softmax 置信度
          ~/debug_image  sensor_msgs/Image  叠加了检测框与预测结果的可视化图像
          ~/fps          std_msgs/Float32   实测处理帧率（相邻拍时间差的滑动平均）
    订阅：/mnist_digit_control/action std_msgs/String 控制节点反馈的动作名（用于叠加显示）

参数：
    source         视频源类型：video | webcam | images
    source_path    视频文件路径或图片目录（source=video/images 时有效）
    model_path     CNN 权重路径（默认取包内 models/mnist_cnn.pth）
    conf_threshold 稳定判定所需的最小置信度（供下游防抖参考）
    display        是否弹出 OpenCV 可视化窗口
"""
import os
import time

import cv2
import numpy as np
import torch
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String

from .model import load_model
from .preprocess import extract_digit

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class DigitDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("digit_detector")
        self.declare_parameter("source", "video")
        self.declare_parameter("source_path", "")
        self.declare_parameter("model_path", os.path.join(PKG_DIR, "models", "mnist_cnn.pth"))
        self.declare_parameter("conf_threshold", 0.7)
        self.declare_parameter("display", True)
        self.declare_parameter("rate", 10.0)

        self.source = self.get_parameter("source").value
        self.source_path = self.get_parameter("source_path").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.display = bool(self.get_parameter("display").value)
        rate = float(self.get_parameter("rate").value)

        model_path = self.get_parameter("model_path").value
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"未找到模型权重 {model_path}，请先运行 python3 main.py train 训练"
            )
        self.model = load_model(model_path)
        self.get_logger().info(f"已加载模型 {model_path}")

        self.bridge = CvBridge()
        self.pub_digit = self.create_publisher(Int32, "~/digit", 10)
        self.pub_conf = self.create_publisher(Float32, "~/confidence", 10)
        self.pub_fps = self.create_publisher(Float32, "~/fps", 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_debug = self.create_publisher(Image, "~/debug_image", qos)

        self.action_label = ""
        self.create_subscription(String, "/mnist_digit_control/action", self._on_action_feedback, 10)

        self.capture = self._open_source()
        self.infer_ms = 0.0
        self._last_tick = None      # 实测处理帧率用上一拍时间戳
        self.meas_hz = 0.0
        self.create_timer(1.0 / rate, self.tick)

    # ------------------------------------------------------------------ source
    def _open_source(self):
        if self.source == "webcam":
            cap = cv2.VideoCapture(0)
        elif self.source == "video":
            if not self.source_path or not os.path.isfile(self.source_path):
                raise FileNotFoundError(f"视频文件不存在: {self.source_path}")
            cap = cv2.VideoCapture(self.source_path)
        elif self.source == "images":
            if not self.source_path or not os.path.isdir(self.source_path):
                raise FileNotFoundError(f"图片目录不存在: {self.source_path}")
            cap = _ImageSeqReader(self.source_path)
        else:
            raise ValueError(f"未知视频源类型: {self.source}")
        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频源: {self.source}:{self.source_path}")
        return cap

    # ---------------------------------------------------------------- feedback
    def _on_action_feedback(self, msg: String) -> None:
        self.action_label = msg.data

    # -------------------------------------------------------------------- loop
    def tick(self) -> None:
        # 实测处理帧率：相邻两拍时间差的滑动平均（比理论 rate 更真实）
        now = time.monotonic()
        if self._last_tick is not None:
            dt = now - self._last_tick
            if dt > 0:
                hz = 1.0 / dt
                self.meas_hz = hz if self.meas_hz <= 0 else 0.9 * self.meas_hz + 0.1 * hz
        self._last_tick = now

        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warn("视频源读取结束，节点退出")
            raise SystemExit(0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        t0 = time.perf_counter()
        digit28, box = extract_digit(gray)
        digit, conf = -1, 0.0
        if digit28 is not None:
            digit, conf = self._infer(digit28)
        self.infer_ms = (time.perf_counter() - t0) * 1000.0

        self.pub_digit.publish(Int32(data=digit))
        self.pub_conf.publish(Float32(data=conf))
        self.pub_fps.publish(Float32(data=self.meas_hz))

        debug = self._draw(frame, digit, conf, box)
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        if self.display:
            cv2.imshow("detector", debug)
            cv2.setWindowTitle("detector", f"digit={digit} conf={conf:.2f} {self.action_label}")
            cv2.waitKey(1)

    def _infer(self, digit28: np.ndarray):
        x = torch.from_numpy(digit28.astype(np.float32)[None, None] / 255.0)
        with torch.no_grad():
            logits = self.model(x)
            probs = torch.softmax(logits, dim=1)[0]
        conf, idx = float(probs.max()), int(probs.argmax())
        return idx, conf

    # -------------------------------------------------------------------- draw
    def _draw(self, frame, digit: int, conf: float, box) -> np.ndarray:
        debug = frame.copy()
        if box is not None:
            x, y, w, h = box
            color = (0, 200, 0) if (digit >= 0 and conf >= self.conf_threshold) else (0, 140, 255)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
        text = f"digit: {digit if digit >= 0 else '-'}  conf: {conf:.2f}"
        cv2.putText(debug, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
        cv2.putText(debug, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        if self.action_label:
            cv2.putText(debug, self.action_label, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(debug, self.action_label, (8, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1)
        return debug


class _ImageSeqReader:
    """按文件名顺序读取图片目录，模拟视频流。"""

    def __init__(self, folder: str) -> None:
        exts = (".png", ".jpg", ".jpeg", ".bmp")
        self.files = sorted(f for f in os.listdir(folder) if f.lower().endswith(exts))
        if not self.files:
            raise FileNotFoundError(f"目录中没有图片: {folder}")
        self.folder = folder
        self.idx = 0

    def isOpened(self) -> bool:  # noqa: N802（与 cv2.VideoCapture 接口对齐）
        return True

    def read(self):
        if self.idx >= len(self.files):
            return False, None
        img = cv2.imread(os.path.join(self.folder, self.files[self.idx]))
        self.idx += 1
        return (img is not None), img

    def release(self) -> None:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigitDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
