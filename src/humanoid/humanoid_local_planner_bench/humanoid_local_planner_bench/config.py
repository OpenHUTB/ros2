# -*- coding: utf-8 -*-
"""全局参数。"""

VERSION = "1.0.0"

CONTROL_HZ = 20.0          # 规划频率 (Hz)
N_RAYS = 72                # 激光束数（5°间隔，避免 2 m 外点间距大于机器人直径而漏检）
RAY_RANGE = 4.0            # 激光量程 (m)
SCAN_HEIGHT = 0.80         # 激光发射点离地高度 (m)，ros_node 用它发布静态 TF
FOOTPRINT_R = 0.28         # 人形足迹半径 (m)，对应肩宽 0.45 + 裕度

START = (0.0, 0.0, 0.0)    # 起点 (x, y, yaw)
GOAL = (8.0, 0.0)          # 终点 (x, y)
TIME_LIMIT = 60.0          # 单次实验时长上限 (s)
GOAL_TOL = 0.35            # 到达判定半径 (m)
DEFAULT_OUTPUT_DIR = "results"
