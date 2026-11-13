#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROV 六推进器推力分配（纯 Python + numpy，不依赖 ROS）。

给定期望的六维力/力矩（机体坐标系，单位 N 与 N·m）：

    wrench = [Fx, Fy, Fz, Mx, My, Mz]

按照推进器布置矩阵 B（6 x N），用最小二乘伪逆求每个推进器的推力：

    T = B^+ * wrench

并处理两类工程问题：

1. **饱和**：任一推进器超过最大推力时，
   - `clamp` 模式直接限幅（会改变合力的方向）；
   - `scale` 模式把整个期望力/力矩按比例缩小（方向不变，但幅度变小）；
2. **推进器故障**：把故障推进器对应的列从 B 中移除，用剩余推进器重新分配，
   故障推进器输出固定为 0。

推进器布置参考 BlueROV2 一类的六推进器 ROV：
4 个 45° 布置的水平推进器 + 2 个垂直推进器。该布置无法产生横滚力矩（Mx），
因此 B 的秩为 5，横滚是不可控自由度。
"""
from __future__ import print_function

import numpy as np

# 推进器布置：名称、安装位置 (x, y) m、推力方向（单位向量，机体坐标系）
THRUSTERS = [
    {'name': 'T1_front_left', 'pos': (0.22, 0.16), 'dir': (0.7071, 0.7071, 0.0)},
    {'name': 'T2_front_right', 'pos': (0.22, -0.16), 'dir': (0.7071, -0.7071, 0.0)},
    {'name': 'T3_rear_left', 'pos': (-0.22, 0.16), 'dir': (-0.7071, 0.7071, 0.0)},
    {'name': 'T4_rear_right', 'pos': (-0.22, -0.16), 'dir': (-0.7071, -0.7071, 0.0)},
    {'name': 'T5_vertical_front', 'pos': (0.18, 0.0), 'dir': (0.0, 0.0, 1.0)},
    {'name': 'T6_vertical_rear', 'pos': (-0.18, 0.0), 'dir': (0.0, 0.0, 1.0)},
]

WRENCH_NAMES = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')


def build_allocation_matrix(thrusters=None):
    """由推进器布置构造 6 x N 的分配矩阵 B：wrench = B * T。"""
    if thrusters is None:
        thrusters = THRUSTERS
    B = np.zeros((6, len(thrusters)))
    for i, thruster in enumerate(thrusters):
        x, y = thruster['pos']
        dx, dy, dz = thruster['dir']
        force = np.array([dx, dy, dz], dtype=float)
        moment = np.cross(np.array([x, y, 0.0]), force)
        B[:, i] = np.concatenate([force, moment])
    return B


def allocate(wrench, max_thrust=20.0, failed=None, mode='scale', thrusters=None):
    """把期望 wrench 分配到各推进器。

    参数
    ----
    wrench : 长度 6 的序列 [Fx, Fy, Fz, Mx, My, Mz]
    max_thrust : 单个推进器最大推力（N）
    failed : 故障推进器下标（从 0 开始），None 表示全部正常
    mode : 'scale'（按比例缩小，保方向）或 'clamp'（直接限幅）
    thrusters : 可选的自定义推进器布置

    返回字典：commands、achieved、residual、saturated、scale、active
    """
    if thrusters is None:
        thrusters = THRUSTERS
    B = build_allocation_matrix(thrusters)
    wrench = np.asarray(wrench, dtype=float)
    n = B.shape[1]

    active = [i for i in range(n) if i != failed]
    B_active = B[:, active]
    raw = np.linalg.pinv(B_active).dot(wrench)

    scale = 1.0
    saturated = False
    if len(raw) and np.max(np.abs(raw)) > max_thrust:
        saturated = True
        if mode == 'clamp':
            raw = np.clip(raw, -max_thrust, max_thrust)
        else:
            scale = max_thrust / np.max(np.abs(raw))
            raw = raw * scale

    commands = np.zeros(n)
    for k, index in enumerate(active):
        commands[index] = raw[k]

    achieved = B.dot(commands)
    residual = wrench - achieved
    return {
        'commands': commands,
        'achieved': achieved,
        'residual': residual,
        'saturated': saturated,
        'scale': scale,
        'active': active,
    }


def residual_ratio(result, wrench):
    """残差相对期望 wrench 的比例（%），用于横向对比不同分配策略。"""
    wrench = np.asarray(wrench, dtype=float)
    norm = np.linalg.norm(wrench)
    if norm < 1e-12:
        return 0.0
    return float(np.linalg.norm(result['residual']) / norm * 100.0)
