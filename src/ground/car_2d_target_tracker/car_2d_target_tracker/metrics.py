#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评价指标（纯函数）。

原实现只能"看窗口"判断追得好不好。这里把可复算的量固定下来：

* ``reached``        是否在时限内进入到达半径
* ``arrive_time``    首次到达的时刻（未到达为 nan）
* ``path_length``    全程走过的路程（px）
* ``min_clearance``  与障碍表面的最小间隙（px，负值表示擦碰）
* ``stuck_fraction`` 到达前"几乎没动"的时间占比（衡量卡死程度）
* ``final_dist``     结束时的目标距离
"""
from __future__ import annotations

import math

import numpy as np


def _arr(x):
    return np.asarray(x, dtype=np.float64).ravel()


def first_reach_time(t, dist, arrive_radius):
    """首次 dist <= arrive_radius 的时刻；未到达返回 nan。"""
    t, d = _arr(t), _arr(dist)
    idx = np.nonzero(d <= float(arrive_radius))[0]
    return float(t[int(idx[0])]) if idx.size else float("nan")


def path_length(x, y):
    x, y = _arr(x), _arr(y)
    if x.size < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def stuck_fraction(t, v, arrive_time, speed_eps):
    """到达前速度低于 eps 的时间占比（衡量"卡在障碍前不动"的程度）。

    未到达时用整段时长做分母：卡死本身就是"全程几乎没动"，应当体现出来。
    """
    t, v = _arr(t), _arr(v)
    if t.size < 2:
        return 0.0
    slow = np.abs(v) < float(speed_eps)
    if math.isnan(arrive_time):
        return float(np.mean(slow))
    mask = t <= float(arrive_time)
    if not np.any(mask):
        return 0.0
    return float(np.mean(slow[mask]))


def summarize(t, x, y, v, dist, clearances, arrive_radius, speed_eps=5.0):
    """把一条轨迹压成指标字典。"""
    t, x, y, v, dist = _arr(t), _arr(x), _arr(y), _arr(v), _arr(dist)
    if t.size == 0:
        raise ValueError("空轨迹")
    arrive = first_reach_time(t, dist, arrive_radius)
    return {
        "reached": bool(not math.isnan(arrive)),
        "arrive_time": arrive,
        "path_length": path_length(x, y),
        "min_clearance": float(np.min(_arr(clearances))) if len(clearances) else float("inf"),
        "stuck_fraction": stuck_fraction(t, v, arrive, speed_eps),
        "final_dist": float(dist[-1]),
    }
