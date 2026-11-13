# -*- coding: utf-8 -*-
"""模块级常量：场景几何、人形足迹、激光、规划器参数、指标阈值。

所有长度单位 m，角度单位 rad，时间单位 s。

本模块唯一被操纵的自变量是 ``INFLATE_*``（代价函数里用的膨胀半径）：
它应当等于「足迹半径 + 安全裕度」。按质点假设取 0.18 m 时，规划器会把
「贴着自己身体擦过去」判成可行解，这正是实验要暴露的问题。
"""

PACKAGE_NAME = "humanoid_passage_width_sweep"
VERSION = "1.0.0"
MODEL_NAME = "corridor_scene.xml"

# ---------------------------------------------------------------- 场景
CORRIDOR_HALF_WIDTH = 0.90
CORRIDOR_LENGTH = 10.0
CORRIDOR_X_MID = 4.0
WALL_THICKNESS = 0.10
WALL_HEIGHT = 1.20
GATE_X = 4.0
DEFAULT_WIDTH = 0.90

# 宽度扫描档位：0.62 只比足迹直径(0.60)宽 2 cm；1.10 很宽松
WIDTH_GRID = (0.62, 0.70, 0.78, 0.86, 0.94, 1.02, 1.10)

# 两个圆柱障碍故意错开中心线（正压在中心线上会让弹性带/采样规划退化成奇异）。
# 半径 0.22 m，机器人要过必须让出约 0.5 m 的横向空间。
OBSTACLES = ((2.60, 0.35), (6.00, -0.35))
OBSTACLE_RADIUS = 0.22

START = (0.0, 0.0, 0.0)
GOAL = (8.0, 0.0)
GOAL_RADIUS = 0.30

# ---------------------------------------------------------------- 人形足迹
FOOTPRINT_RADIUS = 0.30         # 碰撞圆柱半径（肩宽 0.45 m 外扩后取整）
FOOTPRINT_HEIGHT = 1.70
SAFETY_MARGIN = 0.08            # 代价函数里的安全裕度

INFLATE_FOOTPRINT = FOOTPRINT_RADIUS + SAFETY_MARGIN   # 0.38，足迹感知
INFLATE_POINT = 0.18                                  # 质点假设（故意偏小）

# ---------------------------------------------------------------- 激光
N_RAYS = 64
RAY_RANGE = 4.0
RAY_Z = 0.35

# ---------------------------------------------------------------- 运动学
TIMESTEP = 0.01
CONTROL_HZ = 20.0
MAX_V = 0.60
MAX_W = 1.20

# ---------------------------------------------------------------- MPPI
MPPI_SAMPLES = 300
MPPI_HORIZON = 30
MPPI_DT = 0.06
MPPI_TEMP = 0.55
MPPI_SIGMA_V = 0.22
MPPI_SIGMA_W = 0.55
MPPI_W_OBS = 40.0
MPPI_W_PROGRESS = 6.0
MPPI_W_HEADING = 0.7

# ---------------------------------------------------------------- 实验
TIME_LIMIT = 60.0
SEED = 20261112                 # MPPI 采样种子，结果可复现
REPEATS = 2                     # 有随机性，每档重复次数

# ---------------------------------------------------------------- 指标阈值
