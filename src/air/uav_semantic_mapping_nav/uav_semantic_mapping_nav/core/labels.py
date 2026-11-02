#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义类别表：与底座 ``uav_mujoco_sim.core.sensors.SEMANTIC_CLASSES`` 的 id 对齐。

| id | 名称 | 可通行性代价 | 说明 |
|---|---|---|---|
| 0 | 背景 | 20（高）| 无回波 / 未分类 |
| 1 | 地面 | 1.0（基准）| 主要通行区域 |
| 2 | 建筑 | 20（高）| 墙、房屋 |
| 3 | 植被 | 3.0 | 可穿越但代价高 |
| 4 | 障碍 | 20（高）| 一般障碍物 |
| 5 | 门 | 1.2 | 竞速场景的门框 |
| 6 | 车辆 | 20（高）| |
| 7 | 行人 | 20（高）| |
| 8 | 自身 | 1.0 | 无人机机体，建图时不入图 |

**只有几何占据给 ``inf``。** 语义是网络估计，把估计当硬约束会让一次误分类
直接堵死通路——实测网络把场景起点格判成"建筑"、置信度 1.00，语义代价图立刻
无解（成功率 0%，几何图 100%）。不可通行的类别给一个大而有限的值：能绕就绕，
绕不开也能穿过去，代价记在"路径平均语义代价"里。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

__all__ = ["N_CLASS", "SEMANTIC_NAMES", "SEMANTIC_RGB", "TRAVERSAL_COST",
           "BLOCKED", "CLASS_NAMES_CN"]

N_CLASS = 9

SEMANTIC_NAMES = ["background", "ground", "building", "vegetation", "obstacle",
                  "gate", "vehicle", "person", "drone"]

CLASS_NAMES_CN = ["背景", "地面", "建筑", "植被", "障碍", "门", "车辆",
                  "行人", "自身"]

#: 可视化配色（RGB 0~1），与底座分割渲染的配色接近，便于对照
SEMANTIC_RGB = np.array([
    [0.10, 0.10, 0.12],      # 背景
    [0.55, 0.52, 0.45],      # 地面
    [0.75, 0.72, 0.68],      # 建筑
    [0.18, 0.55, 0.24],      # 植被
    [0.85, 0.45, 0.10],      # 障碍
    [0.85, 0.80, 0.20],      # 门
    [0.30, 0.45, 0.80],      # 车辆
    [0.85, 0.25, 0.55],      # 行人
    [0.95, 0.95, 0.95],      # 自身
], dtype=float)

#: 单格通行代价倍率（语义给出）。几何占据另由 ``inf`` 覆盖，见 ``cost_map``。
TRAVERSAL_COST = np.array([20.0, 1.0, 20.0, 3.0, 20.0, 1.2,
                           20.0, 20.0, 1.0], dtype=float)

#: 语义上"不该飞进去"的类别：代价高但不是硬约束
BLOCKED = np.array([True, False, True, False, True, False,
                    True, True, False])
