# -*- coding: utf-8 -*-
"""可视化与仿真参数。"""

import math

# 画布
WIDTH = 900
HEIGHT = 600
FPS = 60

# 颜色
BG_COLOR = (24, 28, 38)
GRID_COLOR = (40, 46, 60)
START_COLOR = (100, 240, 130)        # 起点（绿色）
END_COLOR = (240, 100, 100)          # 终点（红色）
SHORTEST_COLOR = (80, 200, 240)      # 最短路径高亮（青色）
CANDIDATE_COLORS = [                 # 6 条候选路径（淡色）
    (90, 100, 120),
    (110, 120, 140),
    (130, 140, 160),
    (150, 160, 180),
    (170, 180, 200),
    (190, 200, 220),
]
CENTER_COLOR = (200, 120, 240)       # 圆心（紫色，调试用）
TEXT_COLOR = (220, 225, 235)
HUD_COLOR = (200, 210, 225)

# Dubins 默认位姿（起点 + 终点）
DEFAULT_START = (150.0, 480.0, 0.0)            # (x, y, yaw) yaw 单位弧度
DEFAULT_END = (750.0, 150.0, math.pi)          # 朝向 π（向左）
DEFAULT_RADIUS = 80.0                          # 最小转弯半径（像素）

# 采样
CURVE_SAMPLES = 200        # 路径采样点数
ANIM_STEPS = 120           # 绘制动画步数

# 录制
RECORD_EVERY = 2           # 每隔 N 步存一帧
RECORD_TAIL = 30           # 动画结束后补若干帧
RECORD_SCALE = 0.75        # 缩放比例控制 gif 体积
