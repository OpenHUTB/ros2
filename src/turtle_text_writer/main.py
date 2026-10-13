#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
turtle_text_writer - 小海龟写字机
输入一段文字，让 turtlesim 的小海龟把它写出来。
用法:
    直接运行:  python3 main.py --text "HELLO"
    一键启动:  ros2 launch launch/turtle_writer.launch.py text:="HELLO"
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen, TeleportAbsolute

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("缺少 Pillow，请先运行: sudo apt install -y python3-pil fonts-noto-cjk")
    sys.exit(1)

# 常见中英文字体（按顺序找第一个存在的）
FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/ukai.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]

PEN_COLOR = (255, 255, 255)   # 画笔颜色（白色，深色背景上清晰）
CANVAS_W = 220               # 文字位图宽度（像素）
CANVAS_H = 110               # 文字位图高度（像素）


def pick_font(text, canvas_w, canvas_h):
    """找一个能装下整段文字的字体，找不到合适的字号就报错"""
    for path in FONT_CANDIDATES:
        if not os.path.exists(path):
            continue
        for size in (72, 56, 44, 34, 26, 20, 14, 10):
            try:
                font = ImageFont.truetype(path, size)
                probe = Image.new("L", (10, 10), 255)
                bbox = ImageDraw.Draw(probe).textbbox((0, 0), text, font=font)
                w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
                if w <= canvas_w * 0.98 and h <= canvas_h * 0.9:
                    return font, bbox
            except Exception:
                continue
    raise RuntimeError(
        "找不到能容纳这段文字的字体/字号，请检查是否安装: sudo apt install -y fonts-noto-cjk"
    )


class TurtleTextWriter(Node):
    def __init__(self, text):
        super().__init__("turtle_text_writer")
        self.text = text
        self.cli_tp = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.cli_pen = self.create_client(SetPen, "/turtle1/set_pen")

    def wait_services(self):
        while not self.cli_tp.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("等待 turtlesim 服务...")
        while not self.cli_pen.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("等待画笔服务...")

    def _call(self, cli, request):
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def _pen(self, off=False):
        req = SetPen.Request()
        req.r, req.g, req.b = PEN_COLOR
        req.width = 3
        req.off = 1 if off else 0
        self._call(self.cli_pen, req)

    def _move(self, x, y):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        self._call(self.cli_tp, req)

    def _render_segments(self):
        """把文字渲染成位图，再提取成 '行号 -> 水平线段' 的列表"""
        font, bbox = pick_font(self.text, CANVAS_W, CANVAS_H)
        img = Image.new("L", (CANVAS_W, CANVAS_H), 255)
        draw = ImageDraw.Draw(img)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        x0 = (CANVAS_W - tw) / 2 - bbox[0]
        y0 = (CANVAS_H - th) / 2 - bbox[1]
        draw.text((x0, y0), self.text, font=font, fill=0)

        segments = []  # (row_y, [(x_start, x_end), ...])
        for y in range(CANVAS_H):
            black = [x for x in range(CANVAS_W) if img.getpixel((x, y)) < 128]
            runs = []
            start = prev = None
            for x in black:
                if start is None:
                    start = x
                elif x - prev > 1:
                    runs.append((start, prev))
                    start = x
                prev = x
            if start is not None:
                runs.append((start, prev))
            segments.append((y, runs))
        return segments

    def draw(self):
        self.get_logger().info("开始书写: %s" % self.text)
        self.wait_services()
        segments = self._render_segments()

        # 图像坐标 -> turtlesim 画布坐标（画布 0~11，y 轴向上）
        scale_x = 10.0 / CANVAS_W
        scale_y = 9.5 / CANVAS_H
        ox, oy = 0.5, 11.0

        def to_turtle(px, py):
            return ox + px * scale_x, oy - py * scale_y

        # 先抬笔飞到起点
        sx, sy = to_turtle(0, 0)
        self._pen(off=True)
        self._move(sx, sy)

        for y, runs in segments:
            if not runs:
                continue
            for xa, xb in runs:
                ax, ay = to_turtle(xa, y)
                bx, by = to_turtle(xb, y)
                self._pen(off=True)
                self._move(ax, ay)
                self._pen(off=False)
                self._move(bx, by)
            time.sleep(0.03)  # 每行停顿一下，让书写过程肉眼可见（录 GIF 更好看）

        self._pen(off=True)
        self.get_logger().info("书写完成!")


def main(args=None):
    parser = argparse.ArgumentParser(description="小海龟写字机: 让 turtlesim 小海龟书写文字")
    parser.add_argument("--text", default="HUTB", help="要书写的文字, 默认 HUTB")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TurtleTextWriter(cli_args.text)
    try:
        node.draw()
    except Exception as e:
        node.get_logger().error("书写失败: %s" % e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
