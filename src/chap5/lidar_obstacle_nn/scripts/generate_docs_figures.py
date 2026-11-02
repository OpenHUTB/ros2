#!/usr/bin/env python3
"""Generate chapter 5 LiDAR tutorial diagrams as PNG assets."""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[4]
OUTPUT_DIR = REPO_ROOT / 'docs' / 'img' / 'chap5' / 'lidar_obstacle_nn'
GAZEBO_IMAGE = OUTPUT_DIR / 'gazebo_static_2026-11-01.png'

WHITE = '#ffffff'
INK = '#162033'
MUTED = '#5f6f86'
PANEL = '#f4f7fb'
LINE = '#d4deea'
BLUE = '#2563eb'
BLUE_SOFT = '#eaf1ff'
TEAL = '#0f766e'
TEAL_SOFT = '#e7f7f4'
ORANGE = '#c2410c'
ORANGE_SOFT = '#fff1e8'
RED = '#c62828'
RED_SOFT = '#ffebee'
GREEN = '#2e7d32'
GREEN_SOFT = '#e9f6ea'


def find_font(candidates: tuple[str, ...]) -> str:
    for candidate in candidates:
        if Path(candidate).is_file():
            return candidate
    raise FileNotFoundError('No CJK font found: %s' % (candidates,))


CJK_FONT = find_font((
    r'C:\Windows\Fonts\msyh.ttc',
    r'C:\Windows\Fonts\simhei.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
))
CJK_BOLD = find_font((
    r'C:\Windows\Fonts\msyhbd.ttc',
    r'C:\Windows\Fonts\simhei.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
))
MONO_FONT = find_font((
    r'C:\Windows\Fonts\consola.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
))


def font(path: str, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(path, size)


def text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    value: str,
    size: int,
    fill: str = INK,
    bold: bool = False,
    anchor: str = 'la',
    mono: bool = False,
) -> None:
    path = MONO_FONT if mono else CJK_BOLD if bold else CJK_FONT
    draw.text(xy, value, font=font(path, size), fill=fill, anchor=anchor)


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    fill: str = MUTED,
    width: int = 4,
) -> None:
    draw.line((start, end), fill=fill, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    size = 14
    left = (
        end[0] - size * math.cos(angle - math.pi / 6),
        end[1] - size * math.sin(angle - math.pi / 6),
    )
    right = (
        end[0] - size * math.cos(angle + math.pi / 6),
        end[1] - size * math.sin(angle + math.pi / 6),
    )
    draw.polygon((end, left, right), fill=fill)


def node(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    title: str,
    lines: list[str],
    accent: str,
    soft: str,
    tag: str,
) -> None:
    x1, y1, x2, y2 = box
    draw.rounded_rectangle(box, radius=16, fill=soft, outline=accent, width=3)
    draw.rounded_rectangle((x1, y1, x1 + 92, y1 + 34), radius=8, fill=accent)
    text(draw, (x1 + 46, y1 + 17), tag, 16, WHITE, True, 'mm')
    text(draw, (x1 + 24, y1 + 48), title, 27, INK, True)
    for index, value in enumerate(lines):
        text(draw, (x1 + 24, y1 + 88 + index * 34), value, 20, MUTED)


def save(image: Image.Image, name: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    image.save(OUTPUT_DIR / name, optimize=True)


def system_dataflow() -> None:
    image = Image.new('RGB', (1600, 900), WHITE)
    draw = ImageDraw.Draw(image)

    text(draw, (70, 55), '基于激光雷达的神经网络障碍物分类', 42, INK, True)
    text(draw, (72, 112), 'ROS 2 Humble / Gazebo Fortress 数据流', 24, MUTED)
    draw.rounded_rectangle((70, 150, 1530, 226), radius=10, fill=PANEL)
    text(draw, (104, 188), 'GPU LiDAR', 22, BLUE, True, 'lm')
    text(draw, (255, 188), '360 束 · 5 Hz · 0.05–30 m', 21, INK, anchor='lm')
    text(draw, (710, 188), '端到端链路', 21, TEAL, True, 'lm')
    text(draw, (860, 188), 'Gazebo → ros_gz_bridge → MLP → ROS 话题', 21, MUTED, anchor='lm')

    nodes = [
        ((85, 290, 420, 475), 'Gazebo Fortress', [
            'gpu_lidar，360 束全向扫描',
            '遇到障碍产生真实回波',
            'Gazebo Transport 发布 /scan',
        ], BLUE, BLUE_SOFT, '01  仿真'),
        ((465, 290, 800, 475), 'ros_gz_bridge', [
            'gz.msgs.LaserScan',
            '→ sensor_msgs/LaserScan',
            'ROS 2 话题：/scan',
        ], TEAL, TEAL_SOFT, '02  桥接'),
        ((845, 290, 1180, 475), '扫描预处理', [
            'inf / nan → 30 m',
            '重采样为固定 360 维',
            '距离归一化到 0..1',
        ], ORANGE, ORANGE_SOFT, '03  预处理'),
        ((1225, 290, 1515, 475), 'LidarMLP', [
            '360 → 128 → 128 → 3',
            'ReLU + Softmax',
            'CPU 默认，CUDA 可选',
        ], RED, RED_SOFT, '04  推理'),
    ]
    for box, title, lines, accent, soft, tag in nodes:
        node(draw, box, title, lines, accent, soft, tag)

    for start, end in (
        ((420, 382), (465, 382)),
        ((800, 382), (845, 382)),
        ((1180, 382), (1225, 382)),
    ):
        arrow(draw, start, end)

    draw.rounded_rectangle((85, 545, 1515, 665), radius=16, fill='#111827')
    text(draw, (125, 585), 'z = MLP(x)', 30, '#dbeafe', True, mono=True)
    text(draw, (125, 630), 'p = softmax(z)', 26, '#93c5fd', mono=True)
    text(draw, (560, 585), '分类规则', 24, '#f9fafb', True)
    text(draw, (560, 630), 'r_min < 1.0 → near   1.0 ≤ r_min < 3.0 → far   r_min ≥ 3.0 → free',
         22, '#d1d5db', mono=True)

    outputs = [
        ((85, 720, 520, 820), 'near', '最近障碍 < 1.0 m', RED),
        ((555, 720, 990, 820), 'far', '1.0 m ≤ 最近障碍 < 3.0 m', ORANGE),
        ((1025, 720, 1515, 820), 'free', '最近障碍 ≥ 3.0 m', GREEN),
    ]
    for box, title, detail, color in outputs:
        draw.rounded_rectangle(box, radius=14, fill=WHITE, outline=color, width=3)
        text(draw, (box[0] + 24, box[1] + 30), title, 28, color, True)
        text(draw, (box[0] + 150, box[1] + 33), detail, 22, INK)

    arrow(draw, (1370, 475), (1370, 705), fill=MUTED)
    arrow(draw, (223, 475), (223, 705), fill=RED)
    arrow(draw, (774, 475), (774, 705), fill=ORANGE)
    arrow(draw, (1260, 475), (1260, 705), fill=GREEN)

    text(draw, (85, 855), '输出：/obstacle_class  (std_msgs/String)   /obstacle_confidence  (std_msgs/Float32)',
         20, MUTED, mono=True)
    save(image, 'system_dataflow.png')


def classification_zones() -> None:
    image = Image.new('RGB', (1600, 900), WHITE)
    draw = ImageDraw.Draw(image)

    text(draw, (70, 55), 'LiDAR 最近障碍分类区域', 42, INK, True)
    text(draw, (72, 112), '俯视示意（不按真实距离比例绘制）', 24, MUTED)

    panel = (70, 155, 1120, 820)
    draw.rounded_rectangle(panel, radius=16, fill='#1f2937')
    for x in range(110, 1120, 72):
        draw.line((x, 170, x, 805), fill='#374151', width=1)
    for y in range(190, 820, 72):
        draw.line((90, y, 1100, y), fill='#374151', width=1)

    cx, cy = 565, 485
    for radius, fill in ((285, '#173b2d'), (182, '#4a361d'), (62, '#4a232a')):
        draw.ellipse(
            (cx - radius, cy - radius, cx + radius, cy + radius),
            fill=fill,
        )

    for degrees in range(0, 360, 3):
        angle = math.radians(degrees)
        end = (
            cx + int(320 * math.cos(angle)),
            cy + int(320 * math.sin(angle)),
        )
        draw.line((cx, cy, end), fill='#4c7eff', width=1)

    draw.ellipse((cx - 16, cy - 16, cx + 16, cy + 16), fill='#facc15', outline=WHITE, width=3)
    text(draw, (cx, cy + 35), 'lidar_box', 18, '#fef3c7', anchor='ma')

    obstacles = [
        (cx + 54, cy - 30, RED, '0.6 m / near'),
        (cx + 135, cy - 135, GREEN, '1.8 m / far'),
        (cx - 365, cy + 120, BLUE, '5.0 m / free'),
    ]
    for ox, oy, color, label in obstacles:
        draw.rectangle((ox - 22, oy - 22, ox + 22, oy + 22), fill=color, outline=WHITE, width=2)
        draw.line((cx, cy, ox, oy), fill=color, width=4)
        text(draw, (ox, oy - 38), label, 18, WHITE, True, 'ma')

    text(draw, (cx, cy - 76), 'danger < 1 m', 18, '#fecaca', True, 'mm')
    text(draw, (cx, cy - 198), 'far < 3 m', 18, '#fed7aa', True, 'mm')
    text(draw, (cx - 250, cy + 250), 'free ≥ 3 m', 18, '#bbf7d0', True, 'mm')

    legend = (1160, 155, 1530, 820)
    draw.rounded_rectangle(legend, radius=16, fill=PANEL, outline=LINE, width=2)
    text(draw, (1200, 205), '标签规则', 30, INK, True)
    items = [
        (RED, 'near', 'r_min < 1.0 m'),
        (ORANGE, 'far', '1.0 ≤ r_min < 3.0 m'),
        (GREEN, 'free', 'r_min ≥ 3.0 m'),
    ]
    for index, (color, name, rule) in enumerate(items):
        y = 275 + index * 112
        draw.rounded_rectangle((1200, y, 1490, y + 78), radius=10, fill=WHITE)
        draw.rectangle((1220, y + 20, 1258, y + 58), fill=color)
        text(draw, (1280, y + 20), name, 26, color, True)
        text(draw, (1280, y + 52), rule, 18, MUTED)

    draw.rounded_rectangle((1198, 640, 1492, 770), radius=10, fill='#111827')
    text(draw, (1218, 670), 'r_min = min(r_i)', 22, '#dbeafe', mono=True)
    text(draw, (1218, 710), '仅使用有效回波', 20, '#93c5fd')
    text(draw, (1218, 742), 'inf / nan → 30 m', 20, '#93c5fd')

    text(draw, (70, 858), '分类由每帧扫描中的最小有效距离决定；360 束方向不影响阈值公式。',
         20, MUTED)
    save(image, 'classification_zones.png')


def runtime_effect() -> None:
    image = Image.new('RGB', (1800, 1080), WHITE)
    draw = ImageDraw.Draw(image)

    text(draw, (65, 48), 'Gazebo Fortress 实时运行效果', 42, INK, True)
    text(draw, (68, 102), '真实运行截图 + 扫描统计 + 分类结果', 24, MUTED)

    left = (65, 155, 1065, 725)
    draw.rounded_rectangle(left, radius=16, fill=PANEL, outline=LINE, width=2)
    gazebo = Image.open(GAZEBO_IMAGE).convert('RGB')
    gazebo.thumbnail((960, 475), Image.Resampling.LANCZOS)
    gx = left[0] + (left[2] - left[0] - gazebo.width) // 2
    gy = left[1] + 64
    image.paste(gazebo, (gx, gy))
    text(draw, (95, 174), 'Gazebo 实拍', 24, BLUE, True)
    text(draw, (95, 665), '静态 world：红色 0.6 m、绿色 1.8 m、蓝色 5 m',
         20, MUTED)

    right = (1100, 155, 1735, 725)
    draw.rounded_rectangle(right, radius=16, fill='#202938', outline='#202938', width=2)
    text(draw, (1135, 188), '单帧扫描与分类', 26, WHITE, True)

    cx, cy = 1415, 455
    for radius in (55, 135, 230):
        draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius),
                     outline='#46536a', width=1)
    for degrees in range(0, 360, 6):
        angle = math.radians(degrees)
        draw.line(
            (cx, cy, cx + int(230 * math.cos(angle)), cy + int(230 * math.sin(angle))),
            fill='#4c7eff',
            width=1,
        )
    draw.ellipse((cx - 12, cy - 12, cx + 12, cy + 12), fill='#facc15')
    draw.rectangle((cx + 40, cy - 34, cx + 73, cy - 1), fill=RED)
    draw.rectangle((cx + 105, cy - 123, cx + 138, cy - 90), fill=GREEN)
    draw.rectangle((cx - 262, cy + 93, cx - 229, cy + 126), fill=BLUE)
    text(draw, (cx + 75, cy - 62), '0.400 m', 18, '#fecaca', True)
    text(draw, (cx + 158, cy - 115), '1.8 m', 18, '#bbf7d0', True)
    text(draw, (cx - 215, cy + 143), '5.0 m', 18, '#bfdbfe', True)

    draw.rounded_rectangle((1135, 600, 1700, 690), radius=10, fill='#111827')
    text(draw, (1160, 625), 'near', 32, '#f87171', True)
    text(draw, (1280, 629), 'confidence = 0.978', 24, '#f9fafb', mono=True)

    metrics = [
        ('360', '扫描束'),
        ('80', '有效回波'),
        ('0.400 m', '最近距离'),
        ('4.757 m', '最远回波'),
        ('4.3 Hz', '/scan 频率'),
        ('near', '分类结果'),
        ('0.978', '置信度'),
    ]
    x = 65
    widths = [220, 220, 245, 245, 235, 225, 225]
    for (value, label), width in zip(metrics, widths):
        draw.rounded_rectangle((x, 790, x + width, 930), radius=12, fill=PANEL, outline=LINE, width=2)
        text(draw, (x + width // 2, 830), value, 29, BLUE if value not in ('near',) else RED,
             True, 'ma')
        text(draw, (x + width // 2, 885), label, 19, MUTED, anchor='ma')
        x += width + 16

    text(draw, (65, 1005), '采集环境：Ubuntu 20.04 + ROS 2 Humble + Gazebo Fortress 6.17.1 + CPU PyTorch',
         19, MUTED)
    save(image, 'runtime_effect.png')


def main() -> None:
    system_dataflow()
    classification_zones()
    runtime_effect()
    for name in (
        'system_dataflow.png',
        'classification_zones.png',
        'runtime_effect.png',
    ):
        path = OUTPUT_DIR / name
        print('%s  %d bytes' % (path, path.stat().st_size))


if __name__ == '__main__':
    main()
