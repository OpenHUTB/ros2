#!/usr/bin/env python3
"""无人机摄像头感知节点
订阅 /drone/camera/image_raw，用 YOLOv3-tiny 卷积神经网络（OpenCV DNN 加载
darknet 权重）对图像做目标检测，再对检测框做红色颜色验证（无人机追踪的红色
目标柱），发布目标归一化位置到 /drone/target_pos。

端到端感知链路：输入图像 -> YOLOv3-tiny CNN 检测 -> 红色目标位置 -> 控制。
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

try:
    from ament_index_python.packages import get_package_share_directory
    _BASE = os.path.join(get_package_share_directory('drone_gazebo_sim'), 'models')
except Exception:
    # 源码直接运行时（未 source 安装）退回源码目录
    _BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'models')
_CFG = os.path.join(_BASE, 'yolov3-tiny.cfg')
_WEIGHTS = os.path.join(_BASE, 'yolov3-tiny.weights')

# COCO 80 类类别名（YOLOv3-tiny 训练类别，仅用于日志标注）
_COCO = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli',
    'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

_CONF = 0.3      # 置信度阈值
_NMS = 0.4       # 非极大值抑制阈值
_RED_MIN = 0.15  # 检测框内红色像素最小占比（用于红色目标验证）


class DronePerception(Node):
    """使用 YOLOv3-tiny CNN 对摄像头图像做目标检测并输出红色目标位置。"""

    def __init__(self):
        super().__init__('drone_perception')
        if not os.path.exists(_CFG) or not os.path.exists(_WEIGHTS):
            self.get_logger().error(
                f'模型文件缺失: {_CFG} / {_WEIGHTS}，请确认 models 目录已包含权重')
            raise RuntimeError('CNN 模型文件缺失')

        self.net = cv2.dnn.readNetFromDarknet(_CFG, _WEIGHTS)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        _ln = self.net.getLayerNames()
        self._out = [_ln[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.get_logger().info(
            '无人机感知节点启动：YOLOv3-tiny CNN 目标检测，输出层=%s' % self._out)

        self.sub = self.create_subscription(
            Image, '/drone/camera/image_raw', self.img_cb, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/drone/target_pos', 10)

    # ------------------------------------------------------------------ #
    def _decode(self, outs, w, h):
        """YOLO 后处理：把网络输出解码为边界框 (x,y,w,h)、置信度、类别。"""
        boxes, confs, cls_ids = [], [], []
        for out in outs:
            for det in out:
                scores = det[5:]
                cls = int(np.argmax(scores))
                conf = float(scores[cls])
                if conf < _CONF:
                    continue
                cx, cy, bw, bh = det[:4] * np.array([w, h, w, h])
                x, y = cx - bw / 2.0, cy - bh / 2.0
                boxes.append([int(x), int(y), int(bw), int(bh)])
                confs.append(conf)
                cls_ids.append(cls)
        return boxes, confs, cls_ids

    def _is_red(self, img, box):
        """对检测框区域做 HSV 红色颜色验证，返回红色像素占比。"""
        x, y, bw, bh = box
        x, y = max(0, x), max(0, y)
        x2, y2 = min(img.shape[1], x + bw), min(img.shape[0], y + bh)
        if x2 - x < 5 or y2 - y < 5:
            return 0.0
        roi = img[y:y2, x:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        red = (m1 + m2) > 0
        return float(np.mean(red))

    def img_cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if img.ndim == 3 and img.shape[2] == 3 and msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        blob = cv2.dnn.blobFromImage(
            img, 1 / 255.0, (416, 416), (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self._out)

        boxes, confs, cls_ids = self._decode(outs, img.shape[1], img.shape[0])
        idx = cv2.dnn.NMSBoxes(boxes, confs, _CONF, _NMS)

        out = Float32MultiArray()
        best = None  # (cx_norm, area) 红色占比最高的框
        if len(idx) > 0:
            for i in idx.flatten():
                ratio = self._is_red(img, boxes[i])
                if ratio < _RED_MIN:
                    continue
                area = float(boxes[i][2] * boxes[i][3] / (img.shape[1] * img.shape[0]))
                cx = float((boxes[i][0] + boxes[i][2] / 2.0) / img.shape[1] - 0.5)
                if best is None or ratio > best[2]:
                    best = (cx, area, ratio)
                self.get_logger().debug(
                    '检测: %s conf=%.2f red=%.2f cx=%.2f' %
                    (_COCO[cls_ids[i]] if cls_ids[i] < len(_COCO) else '?',
                     confs[i], ratio, cx))
        if best is not None:
            out.data = [float(best[0]), float(best[1])]
        else:
            out.data = [999.0, 0.0]  # 未找到红色目标
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DronePerception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
