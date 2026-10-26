# -*- coding: utf-8 -*-
"""捕获点（Capture Point / 发散运动分量）判据。

改写自（Apache License 2.0）：
    PrageethM1702/Bipedal-Walking-and-Push-Recovery
    https://github.com/PrageethM1702/Bipedal-Walking-and-Push-Recovery
    （原文件 ``balance/capture_point.py``，Copyright (c) the original authors）

本站改动：
  * 入参由「水平二维位置/速度 + 质心高度」放宽为「任意维位置/速度向量
    （取前两维）+ 质心高度」，便于直接传入本模型 ``read_state()`` 的三维量；
  * 增加 ``capture_point_offset`` 供文档与测试引用。

理论来源：Pratt et al., "Capture Point: A Step toward Humanoid Push Recovery",
IEEE-RAS Humanoids 2006。捕获点 ξ = x_CoM + v_CoM / ω，ω = √(g / z_CoM)：
若 ξ 落在支撑多边形内，仅靠踝 / 髋策略即可停下；落在外面则必须跨步。
"""

import numpy as np

#: 重力加速度 (m/s²)
G = 9.81


def omega(z_com, g=G):
    """线性倒立摆（LIPM）的自然频率 ω = √(g / z_com)，单位 rad/s。"""
    return np.sqrt(g / max(float(z_com), 1e-3))


def capture_point(com, com_vel, height, g=G):
    """捕获点水平位置 (x, y)，单位 m。

    参数:
        com:    质心位置（取前两维为水平位置，m）
        com_vel:质心速度（取前两维为水平速度，m/s）
        height: 质心高度 z_com (m)
    """
    com = np.asarray(com, dtype=float)
    com_vel = np.asarray(com_vel, dtype=float)
    return com[:2] + com_vel[:2] / omega(height, g)


def capture_point_offset(com_vel, height, g=G):
    """捕获点相对质心投影的速度相关偏置 ξ − x_CoM，单位 m。"""
    com_vel = np.asarray(com_vel, dtype=float)
    return com_vel[:2] / omega(height, g)
