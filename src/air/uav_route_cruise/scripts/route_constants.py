#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""route_env / route_node / policy 共用的观测常量.

单独拆出来的原因：部署侧（``route_node.py`` + ``policy.py``）只需要这几个
量纲常量，不应该为了 import 它们而依赖训练环境 ``route_env.py`` 里的
``gymnasium``。文档 ``docs/air/uav_route_cruise.md`` 写明“只部署只需要
numpy”，常量拆出来后这个约定才成立。
"""

# ---- 与 uav_ppo_nav.policy 保持一致的量纲 ----
N_SECTORS = 24
MAX_RANGE = 12.0

# ---- 本模块新增 ----
CROSS_SCALE = 4.0          # 横向偏差归一化尺度 (m)
LOOKAHEAD_SCALE = 6.0      # 前方航点相对位置归一化尺度 (m)
Z_SCALE = 2.0              # 高度偏差归一化尺度 (m)

OBS_DIM = N_SECTORS + 6    # 24 激光 + 6 航线相关 = 30
