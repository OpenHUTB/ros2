# -*- coding: utf-8 -*-
"""detector_node.py —— 感知节点：图像 -> 目标像素 + 可见概率。

订阅 /uav/image (sensor_msgs/Image, rgb8)，发布 /uav/detection
(geometry_msgs/PointStamped: x=u 像素, y=v 像素, z=可见概率)。

权重默认从 install 后的 share/uav_camera_tracking/models/detector.npz 读取，
可用参数 model 覆盖；找不到时给出明确报错（而不是静默输出 0）。
"""
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from .detector import DetectorNet
from .model_io import model_path


class DetectorNode(Node):
    def __init__(self):
        super().__init__('uav_detector')
        default_model = model_path('detector.npz')
        self.declare_parameter('model', default_model)
        self.declare_parameter('vis_thresh', 0.5)
        self.declare_parameter('report_every', 100)

        self.vis_thresh = float(self.get_parameter('vis_thresh').value)
        self.report_every = int(self.get_parameter('report_every').value)
        path = str(self.get_parameter('model').value)
        if not os.path.isfile(path):
            raise FileNotFoundError(
                '感知网络权重不存在：%s\n'
                '请先训练（ros2 run uav_camera_tracking main）或用参数 model 指定路径。'
                % path)
        self.detector = DetectorNet.load(path)
        self.get_logger().info('感知网络已加载：%s' % path)

        self.n = 0
        self.dt_sum = 0.0
        self.pub = self.create_publisher(PointStamped, '/uav/detection', 10)
        self.create_subscription(Image, '/uav/image', self._on_image, 1)

    def _to_float(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding.lower() in ('rgb8', 'bgr8'):
            img = buf.reshape(msg.height, msg.width, 3).astype(np.float32)
            if msg.encoding.lower() == 'bgr8':
                img = img[:, :, ::-1]
        else:                       # mono8 等：复制成三通道
            img = np.repeat(buf.reshape(msg.height, msg.width, 1)
                            .astype(np.float32), 3, axis=2)
        return img / 255.0

    def _on_image(self, msg):
        t0 = time.time()
        img = self._to_float(msg)
        det = self.detector.detect(img)
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.point.x = float(det['u'])
        out.point.y = float(det['v'])
        out.point.z = float(det['vis'])
        self.pub.publish(out)

        self.n += 1
        self.dt_sum += time.time() - t0
        if self.report_every > 0 and self.n % self.report_every == 0:
            self.get_logger().info(
                '已处理 %d 帧，平均单帧 %.2f ms（含解码），最近一次 u=%.1f v=%.1f vis=%.2f'
                % (self.n, 1000.0 * self.dt_sum / self.n,
                   det['u'], det['v'], det['vis']))


def main(argv=None):
    rclpy.init(args=argv)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
