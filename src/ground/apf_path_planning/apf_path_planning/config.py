# -*- coding: utf-8 -*-
"""APF 可视化参数与默认场景。"""

import math

# 画布
WIDTH = 800
HEIGHT = 600
FPS = 30

# 世界坐标范围（与画布 1:1，单位 = 像素）
WORLD_MIN = (0.0, 0.0)
WORLD_MAX = (float(WIDTH), float(HEIGHT))

# 颜色
BG = (18, 22, 28)
GRID = (28, 34, 42)
OBSTACLE = (220, 80, 70)
OBSTACLE_EDGE = (255, 120, 100)
GOAL = (90, 220, 120)
START = (90, 170, 240)
PATH = (90, 200, 240)
FORCE = (255, 220, 80)
HUD = (220, 226, 232)
POTENTIAL_LOW = (30, 40, 60)
POTENTIAL_HIGH = (180, 60, 50)

# 默认场景（世界坐标）
START_POSE = (60.0, 60.0)
GOAL_POSE = (720.0, 540.0)
OBSTACLES = [
    # (x, y, radius)
    (250.0, 200.0, 45.0),
    (400.0, 350.0, 55.0),
    (550.0, 200.0, 40.0),
    (300.0, 450.0, 50.0),
    (600.0, 420.0, 45.0),
]

# APF 算法参数
APF_K_ATT = 1.0
APF_K_REP = 5000.0
APF_D_THRESH_ATT = 150.0
APF_D_THRESH_REP = 80.0
APF_STEP_SIZE = 4.0
APF_MAX_ITER = 2000
APF_GOAL_TOL = 10.0
APF_ESCAPE_JITTER = 15.0
APF_ESCAPE_PATIENCE = 20

# 录制
RECORD_FPS = 30
RECORD_FRAMES = 120
