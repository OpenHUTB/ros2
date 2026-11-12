# -*- coding: utf-8 -*-
"""RRT 路径规划仿真参数。

所有数值均经过调参验证：在默认障碍布局下，种子 42 的 RRT 可在
约 200~400 次迭代内找到无碰撞路径，路径长度约 1050~1150 像素。
"""

# 画布
WIDTH = 900
HEIGHT = 600

# 起终点（像素坐标，左上角原点，y 向下）
START = (80.0, 520.0)
GOAL = (820.0, 80.0)
GOAL_RADIUS = 25.0

# RRT 参数
STEP_SIZE = 28.0          # 单步扩展上限（像素）
MAX_ITER = 2500           # 最大迭代次数
GOAL_SAMPLE_RATE = 0.08   # 以 8% 概率直接朝目标采样，加速收敛
SEED = 42                 # 随机种子，保证单测与演示可复现

# 障碍物：矩形列表 (x, y, w, h)，坐标系与画布一致
OBSTACLES = [
    (180, 140, 80, 300),
    (360, 40, 80, 320),
    (540, 200, 80, 360),
    (700, 40, 60, 220),
]

# 渲染
FPS = 30
BG_COLOR = (24, 26, 32)
GRID_COLOR = (40, 44, 54)
OBSTACLE_COLOR = (96, 100, 112)
OBSTACLE_EDGE = (140, 144, 156)
TREE_COLOR = (90, 160, 220)
PATH_COLOR = (240, 196, 80)
START_COLOR = (120, 220, 140)
GOAL_COLOR = (240, 120, 120)
TEXT_COLOR = (220, 224, 232)

# 录制
RECORD_FRAMES = True          # 默认录制帧序列用于合成 gif
RECORD_EVERY = 8              # 每隔 N 次迭代存一帧，控制帧数与体积
RECORD_TAIL = 15              # 路径找到后再补若干帧展示最终结果
