#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AEB 评价指标（纯函数，喂时间序列就能算，不依赖 mujoco）。

只报"刹住了"没有说服力，这里给几个可复算的量：

* ``min_gap``        全程最小净间距（m），< 0 即发生碰撞
* ``impact_speed``   首次接触瞬间的自车速度（m/s），没撞则为 nan
* ``ttc_min``        全程最小 TTC（s），越小越危险
* ``brake_onset``    制动首次介入的时刻（s），nan 表示全程未介入
* ``brake_integral`` 制动力对时间的积分（无量纲），粗略反映"踩了多少刹车"
* ``stop_distance``  从仿真开始到自车首次停住走过的距离（m）

所有函数都是纯函数：单元测试直接喂人造序列验证定义，不需要跑物理仿真。
"""
from __future__ import annotations

import numpy as np


def _arr(x):
    return np.asarray(x, dtype=np.float64).ravel()


def _trapz(y, x):
    """梯形积分（兼容 numpy 2.x 把 trapz 改名为 trapezoid）。"""
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(y, x))
    return float(np.trapz(y, x))


def first_contact(gap):
    """首次 gap <= 0 的下标；没有接触返回 -1。"""
    g = _arr(gap)
    idx = np.nonzero(g <= 0.0)[0]
    return int(idx[0]) if idx.size else -1


def brake_onset(t, brake, eps=1e-9):
    """制动量首次大于 eps 的时刻；全程未介入返回 nan。"""
    t, b = _arr(t), _arr(brake)
    idx = np.nonzero(b > eps)[0]
    return float(t[int(idx[0])]) if idx.size else float("nan")


def stop_distance(x, v, v_stop=0.1):
    """从起点到"自车首次停住"走过的距离；全程没停住则返回 nan。

    取"首次"而不是"最终"：若发生追尾，速度可能被撞回来再次变化，
    那不是我们要的制动距离。
    """
    x, v = _arr(x), _arr(v)
    idx = np.nonzero(np.abs(v) <= float(v_stop))[0]
    if not idx.size:
        return float("nan")
    k = int(idx[0])
    return float(x[k] - x[0])


def summarize(t, gap, v_ego, ttc, brake, x=None, v_stop=0.1):
    """把一条仿真轨迹压成指标字典。

    ``x`` 给出时额外计算 ``stop_distance``；否则该字段为 nan。
    """
    t, gap, v_ego = _arr(t), _arr(gap), _arr(v_ego)
    ttc, brake = _arr(ttc), _arr(brake)
    if t.size == 0:
        raise ValueError("空轨迹")

    k = first_contact(gap)
    collided = k >= 0
    return {
        "min_gap": float(np.min(gap)),
        "collided": bool(collided),
        "impact_speed": float(v_ego[k]) if collided else float("nan"),
        "ttc_min": float(np.min(ttc)),
        "brake_onset": brake_onset(t, brake),
        "peak_brake": float(np.max(brake)),
        "brake_integral": _trapz(brake, t),
        "stop_distance": (stop_distance(x, v_ego, v_stop)
                          if x is not None else float("nan")),
        "final_gap": float(gap[-1]),
        "final_speed": float(v_ego[-1]),
        "stopped": bool(abs(float(v_ego[-1])) <= float(v_stop)),
    }
