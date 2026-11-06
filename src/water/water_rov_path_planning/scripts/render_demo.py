#!/usr/bin/env python3
"""订阅 water_rov_path_planning 节点输出，离线渲染三联演示动图。"""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Path
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.bridge = CvBridge()
        self.heatmap = None
        self.current = None
        self.path_points = []
        self.frames = []
        self.max_frames = 24
        self.create_subscription(Image, '/heatmap', self.on_heatmap, 10)
        self.create_subscription(Image, '/ocean_current', self.on_current, 10)
        self.create_subscription(Path, '/planned_path', self.on_path, 10)
        self.get_logger().info('记录器启动，等待话题数据...')

    def on_heatmap(self, msg):
        try:
            self.heatmap = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'热力图转换失败: {e}')

    def on_current(self, msg):
        try:
            self.current = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'洋流图转换失败: {e}')

    def on_path(self, msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f'收到路径，点数: {len(self.path_points)}')

    def _label(self, draw, w):
        try:
            font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 20)
        except Exception:
            font = ImageFont.load_default()
        draw.text((10, 10), 'HEATMAP', fill=(255, 255, 255), font=font)
        draw.text((w + 10, 10), 'OCEAN CURRENT', fill=(255, 255, 255), font=font)
        draw.text((2 * w + 10, 10), 'PLANNED PATH', fill=(255, 255, 255), font=font)

    def build_frames(self):
        """沿规划路径推进 rov，生成每帧不同的动画。"""
        h = w = 400
        hm = cv2.resize(self.heatmap, (w, h))
        oc = cv2.resize(self.current, (w, h))
        # 网格 100 格、分辨率 0.5 m → 地图边长 50 m
        scale = w / 50.0
        pts = [(int(x * scale), int(y * scale)) for x, y in self.path_points]
        for i in range(self.max_frames):
            path_img = hm.copy()
            for k in range(len(pts) - 1):
                cv2.line(path_img, pts[k], pts[k + 1], (0, 255, 255), 3)
            cv2.circle(path_img, pts[0], 8, (0, 255, 0), -1)
            cv2.circle(path_img, pts[-1], 8, (0, 0, 255), -1)
            # rov 位置：沿折线匀速推进
            t = i / (self.max_frames - 1) * (len(pts) - 1)
            seg = min(int(t), len(pts) - 2)
            f = t - seg
            rover = (
                int(pts[seg][0] + (pts[seg + 1][0] - pts[seg][0]) * f),
                int(pts[seg][1] + (pts[seg + 1][1] - pts[seg][1]) * f),
            )
            cv2.circle(path_img, rover, 10, (255, 0, 255), 2)
            cv2.circle(path_img, rover, 4, (255, 0, 255), -1)
            combined = np.hstack([hm, oc, path_img])
            pil_img = PILImage.fromarray(cv2.cvtColor(combined, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)
            self._label(draw, w)
            self.frames.append(pil_img)
        self.get_logger().info(f'已生成 {len(self.frames)} 帧')

    def save(self, out_path):
        if not self.frames:
            self.get_logger().error('没有录制到任何帧')
            return False
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        self.frames[0].save(
            out_path,
            save_all=True,
            append_images=self.frames[1:],
            duration=200,
            loop=0,
        )
        size_mb = os.path.getsize(out_path) / (1024 * 1024)
        self.get_logger().info(f'动图已保存: {out_path} ({size_mb:.2f} MB, {len(self.frames)} 帧)')
        return True


def main():
    rclpy.init()
    node = DemoRecorder()
    out = os.path.expanduser('~/demo_rov_path_planning.gif')
    try:
        start = time.time()
        while rclpy.ok() and (node.heatmap is None or node.current is None or not node.path_points):
            if time.time() - start > 20:
                node.get_logger().error('20 秒内未收到完整话题数据，请确认 launch 正在运行')
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('话题数据齐备，开始渲染...')
        node.build_frames()
    finally:
        if node.save(out):
            print(f'\n成功！动图已保存到: {out}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
