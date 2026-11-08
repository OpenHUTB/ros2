#!/usr/bin/env python3
# 离线渲染演示 GIF：订阅节点实际输出的三个图像话题与指标话题，
# 抓取真实运行帧，合成三联对比动图（原始 / 增强 / 追踪结果）加指标曲线。
# 用法（launch 保持运行）：
#     python3 render_demo.py 输出路径.gif
# 生成的 GIF 为节点实际数据渲染，非手工绘制。

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from PIL import Image as PILImage
from PIL import ImageDraw

TOPIC_RAW = '/camera/image_raw'
TOPIC_ENHANCED = '/enhanced_image'
TOPIC_TRACKING = '/tracking_result'
TOPIC_METRICS = '/enhancement_metrics'

PANEL_W = 320
PANEL_H = 240
STRIP_H = 90
NUM_FRAMES = 60      # 共渲染 60 帧（launch 重启后立刻运行，取追踪稳定的开头片段）
FRAME_STEP = 2       # 源 20 fps，每 2 帧取 1 帧，导出 10 fps
HISTORY = 90         # 指标曲线长度


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('water_demo_recorder')
        self.bridge = CvBridge()
        self.latest = {}
        self.metrics = [0.0, 0.0, 0.0, 0.0]
        self.hist_contrast = []
        self.hist_conf = []
        # 订阅对象必须保存引用，否则会被回收导致回调不触发
        self._subs = [
            self.create_subscription(
                Image, TOPIC_RAW, lambda m: self._save('raw', m), 10),
            self.create_subscription(
                Image, TOPIC_ENHANCED, lambda m: self._save('enhanced', m), 10),
            self.create_subscription(
                Image, TOPIC_TRACKING, lambda m: self._save('tracking', m), 10),
            self.create_subscription(
                Float32MultiArray, TOPIC_METRICS, self._metrics_cb, 10),
        ]

    def _save(self, key, msg):
        self.latest[key] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _metrics_cb(self, msg):
        if len(msg.data) >= 4:
            self.metrics = list(msg.data[:4])
            self.hist_contrast.append(self.metrics[0])
            self.hist_conf.append(self.metrics[3])


def sparkline(draw, values, x0, y0, x1, y1, color):
    draw.rectangle([x0, y0, x1, y1], outline=(80, 80, 80))
    vals = values[-HISTORY:]
    if len(vals) < 2:
        return
    lo, hi = min(vals), max(vals)
    span = (hi - lo) or 1.0
    w = x1 - x0
    h = y1 - y0
    pts = []
    for i, v in enumerate(vals):
        px = x0 + w * i / (len(vals) - 1)
        py = y1 - h * (v - lo) / span
        pts.append((px, py))
    draw.line(pts, fill=color, width=1)


def compose(rec):
    panels = []
    for key, label in (('raw', 'RAW'), ('enhanced', 'ENHANCED'),
                       ('tracking', 'TRACKING')):
        arr = rec.latest[key][:, :, ::-1]
        im = PILImage.fromarray(arr).resize((PANEL_W, PANEL_H))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, 100, 18], fill=(0, 0, 0))
        d.text((5, 3), label, fill=(255, 255, 255))
        panels.append(im)

    canvas = PILImage.new('RGB', (PANEL_W * 3, PANEL_H + STRIP_H),
                          (24, 24, 24))
    for i, im in enumerate(panels):
        canvas.paste(im, (PANEL_W * i, 0))

    d = ImageDraw.Draw(canvas)
    c, cf, p, conf = rec.metrics
    d.text((10, PANEL_H + 6),
           f'contrast {c:7.1f}  colorfulness {cf:5.1f}  '
           f'PSNR {p:5.1f} dB  confidence {conf:.2f}',
           fill=(220, 220, 220))
    d.text((10, PANEL_H + 28), 'contrast', fill=(80, 200, 120))
    sparkline(d, rec.hist_contrast,
              80, PANEL_H + 30, 460, PANEL_H + STRIP_H - 10,
              (80, 200, 120))
    d.text((490, PANEL_H + 28), 'confidence', fill=(200, 160, 60))
    sparkline(d, rec.hist_conf,
              570, PANEL_H + 30, 950 - 10, PANEL_H + STRIP_H - 10,
              (200, 160, 60))
    return canvas


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'demo.gif'
    rclpy.init()
    rec = DemoRecorder()
    frames = []
    step = 0
    print('等待话题数据（确认 launch 在运行）...')
    wait_count = 0
    while rclpy.ok() and len(frames) < NUM_FRAMES:
        rclpy.spin_once(rec, timeout_sec=0.05)
        wait_count += 1
        if wait_count % 100 == 0 and not all(
                k in rec.latest for k in ('raw', 'enhanced', 'tracking')):
            print(f'已收到话题: {sorted(rec.latest.keys())}，继续等待...')
            continue
        if not all(k in rec.latest for k in ('raw', 'enhanced', 'tracking')):
            continue
        step += 1
        if step % FRAME_STEP:
            continue
        frames.append(compose(rec))
        if len(frames) % 10 == 0:
            print(f'已渲染 {len(frames)}/{NUM_FRAMES} 帧')
    if frames:
        frames[0].save(out, save_all=True, append_images=frames[1:],
                       duration=100, loop=0, optimize=True)
        print(f'已保存 {out}（{len(frames)} 帧，10 fps）')
    else:
        print('没有抓到任何帧，请检查 launch 是否在运行')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
