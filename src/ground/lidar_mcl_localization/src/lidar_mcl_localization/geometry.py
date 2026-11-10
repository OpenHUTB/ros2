# -*- coding: utf-8 -*-
"""二维位姿工具（纯 NumPy，无 ROS 依赖）。

坐标系约定与 ROS 一致：x 向前、y 向左、yaw 逆时针为正，单位米/弧度。
"""

import math

import numpy as np


def wrap_angle(a):
    """把角度归一化到 [-pi, pi)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angles(a):
    """wrap_angle 的数组版本。"""
    a = np.asarray(a, dtype=np.float64)
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(pose, delta):
    """在 pose 上叠加一个局部增量 delta=(ds, dth)，返回新位姿。

    与 ``_se2_step`` 相同，但用中点近似，比"先转后走"更贴近真实曲线运动。
    """
    x, y, th = pose
    ds, dth = delta
    x += ds * math.cos(th + 0.5 * dth)
    y += ds * math.sin(th + 0.5 * dth)
    th = wrap_angle(th + dth)
    return (x, y, th)


def relative_pose(prev, cur):
    """从 prev 到 cur 的相对运动，返回 (ds, dth)。"""
    dx = cur[0] - prev[0]
    dy = cur[1] - prev[1]
    ds = math.hypot(dx, dy)
    if ds < 1e-9:
        return (0.0, wrap_angle(cur[2] - prev[2]))
    # 位移方向相对 prev 航向的夹角
    heading = math.atan2(dy, dx)
    dth = wrap_angle(cur[2] - prev[2])
    # 注意：这里把位移长度与航向变化分开，正是里程计模型需要的两个量
    return (ds, dth)


def pose_error(est, true):
    """估计与真值的误差：位置误差(米) 与 航向误差(弧度)。"""
    pos = math.hypot(est[0] - true[0], est[1] - true[1])
    yaw = abs(wrap_angle(est[2] - true[2]))
    return pos, yaw


def circular_mean(angles, weights=None):
    """角度的加权圆周平均（普通平均在 +pi/-pi 处会错）。"""
    a = np.asarray(angles, dtype=np.float64)
    if weights is None:
        weights = np.ones_like(a)
    s = np.sum(weights * np.sin(a))
    c = np.sum(weights * np.cos(a))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return math.atan2(s, c)


def weighted_pose(particles, weights):
    """粒子集合的加权位姿估计 → (x, y, yaw)。"""
    w = np.asarray(weights, dtype=np.float64)
    total = w.sum()
    if total <= 0.0:
        w = np.ones_like(w) / len(w)
    else:
        w = w / total
    x = float(np.sum(w * particles[:, 0]))
    y = float(np.sum(w * particles[:, 1]))
    yaw = circular_mean(particles[:, 2], w)
    return (x, y, yaw)
