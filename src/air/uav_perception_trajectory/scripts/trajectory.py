#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹生成模块：参数化参考轨迹 ``p_d(t)`` 与其解析导数 ``ṗ_d(t)``。

对应课程任务（2）的「给定轨迹的控制」部分。

本模块为**纯函数**，不依赖 ROS。

设计说明
--------
导数必须**解析给出**，不能用数值差分近似。差分引入的延迟会让控制器的
前馈项相位滞后，在高速轨迹下表现为持续性的跟踪偏差。

三类轨迹
--------
* ``line``    : 直线段，用于验证稳态误差
* ``helix``   : 螺旋线，用于验证三维跟踪
* ``figure8`` : 8 字，用于验证连续变向
"""

from __future__ import annotations

import numpy as np

TRAJ_TYPES = ("line", "helix", "figure8")


def line(t, v=2.0, y0=0.0, z=-5.0):
    """直线段：沿 +x 匀速飞行。"""
    p_d = np.array([v * t, y0, z], dtype=np.float64)
    p_d_dot = np.array([v, 0.0, 0.0], dtype=np.float64)
    return p_d, p_d_dot


def helix(t, radius=3.0, omega=0.4, z=-5.0, climb=0.3):
    """螺旋线：水平画圆同时缓慢上升。

    Parameters
    ----------
    climb : float
        高度变化率 (m/s)，NED 系下取正表示向上（z 变小）。
    """
    p_d = np.array([
        radius * np.cos(omega * t),
        radius * np.sin(omega * t),
        z - climb * t,
    ], dtype=np.float64)
    p_d_dot = np.array([
        -radius * omega * np.sin(omega * t),
        radius * omega * np.cos(omega * t),
        -climb,
    ], dtype=np.float64)
    return p_d, p_d_dot


def figure8(t, radius=5.0, omega=0.4, z=-5.0):
    """8 字轨迹（Gerono 双纽线的参数形式）。

    水平方向 x 为一次谐波、y 为二次谐波，参数周期正好是 8 字的一整圈。
    """
    p_d = np.array([
        radius * np.sin(omega * t),
        (radius / 2.0) * np.sin(2.0 * omega * t),
        z,
    ], dtype=np.float64)
    p_d_dot = np.array([
        radius * omega * np.cos(omega * t),
        radius * omega * np.cos(2.0 * omega * t),
        0.0,
    ], dtype=np.float64)
    return p_d, p_d_dot


_GENERATORS = {
    "line": line,
    "helix": helix,
    "figure8": figure8,
}


def make_traj(kind="figure8", **kwargs):
    """按名称取轨迹生成函数，忽略不适用于该类型的多余参数。"""
    if kind not in _GENERATORS:
        raise ValueError(
            "未知轨迹类型 {!r}，可选：{}".format(kind, ", ".join(TRAJ_TYPES)))

    fn = _GENERATORS[kind]

    # 只保留该函数签名接受的参数，避免把 radius 传给 line 之类
    import inspect
    accepted = set(inspect.signature(fn).parameters) - {"t"}
    filtered = {k: v for k, v in kwargs.items() if k in accepted}

    def wrapped(t):
        return fn(t, **filtered)

    wrapped.kind = kind
    wrapped.params = filtered
    return wrapped


def period(kind, omega=0.4):
    """轨迹的角周期（秒）。用于判断跑了几圈。"""
    if kind == "line":
        return float("inf")
    return 2.0 * np.pi / omega
