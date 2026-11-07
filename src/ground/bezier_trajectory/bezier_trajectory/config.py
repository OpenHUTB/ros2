# -*- coding: utf-8 -*-
"""可视化与仿真参数。"""

# 画布
WIDTH = 900
HEIGHT = 600
FPS = 60

# 颜色
BG_COLOR = (24, 28, 38)
GRID_COLOR = (40, 46, 60)
CONTROL_POLY_COLOR = (90, 100, 120)
CURVE_COLOR = (80, 200, 240)        # Bezier 曲线（青色）
BSPLINE_COLOR = (240, 180, 80)      # B 样条对比（金色）
CONTROL_POINT_COLOR = (240, 100, 100)
ENDPOINT_COLOR = (100, 240, 130)
CURVATURE_COLOR = (200, 120, 240)
HUD_COLOR = (200, 210, 225)
TEXT_COLOR = (220, 225, 235)

# Bezier 曲线默认控制点（3 次）
DEFAULT_CONTROL_POINTS = [
    (120, 480),   # P0 起点
    (250, 150),   # P1
    (650, 150),   # P2
    (780, 480),   # P3 终点
]

# B 样条对比用的控制点（4 个，生成 3 次 B 样条）
BSPLINE_CONTROL_POINTS = [
    (120, 480),
    (300, 180),
    (600, 180),
    (780, 480),
]

# 采样
CURVE_SAMPLES = 200        # 曲线采样点数
CURVATURE_SAMPLES = 80     # 曲率可视化采样

# 录制
RECORD_FRAMES = True
RECORD_EVERY = 2           # 每隔 N 步存一帧
RECORD_TAIL = 30           # 动画结束后补若干帧
RECORD_SCALE = 0.75        # 缩放比例控制 gif 体积

# 动画
ANIM_STEPS = 120           # de Casteljau 动画步数
