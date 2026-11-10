#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平抛弹道解析解：下落时间、前抛量、风偏与散布。

一行摘要：投放物离机后只受重力与常值风，落点由释放时的三维状态唯一
决定——水平速度决定前抛，风决定横偏，竖直速度决定下落时间长短。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BallisticsCfg:
    """投放物与风场的物理参数（全模块共用一份默认值）。"""

    g: float = 9.81             # m/s^2
    wind_coeff: float = 0.8     # 风偏经验系数（投放物被风拖曳的比例）
    sigma_factor: float = 0.15  # 落点散布 σ = factor · h

    def __post_init__(self):
        if not np.isfinite(self.g) or self.g <= 0.0:
            raise ValueError("g must be positive and finite, got %r" % self.g)
        if not np.isfinite(self.wind_coeff) or self.wind_coeff < 0.0:
            raise ValueError("wind_coeff must be non-negative and finite")
        if not np.isfinite(self.sigma_factor) or self.sigma_factor < 0.0:
            raise ValueError("sigma_factor must be non-negative and finite")


def fall_time(h: float, cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """静止投放的下落时间 t_fall = sqrt(2h/g)。"""
    if not np.isfinite(h) or h <= 0.0:
        raise ValueError("drop height must be positive and finite, got %r" % h)
    return math.sqrt(2.0 * h / cfg.g)


def fall_time_from_state(z_rel: float, vz: float,
                         cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """由实际离地高度与竖直速度解下落时间。

    z 轴向上：z(t) = z0 + vz·t − g·t²/2，取 z = 0 的正根。vz 向上会
    拉长下落时间（落点随之前移），向下则缩短；模型仍是同一套无阻力
    假设，偏差交给调用方用接受半径校验。
    """
    if not np.isfinite(z_rel) or not np.isfinite(vz):
        raise ValueError("non-finite release state: z=%r vz=%r" % (z_rel, vz))
    if z_rel <= 0.0:
        raise ValueError("release at/below ground: z_rel=%r" % z_rel)
    disc = vz * vz + 2.0 * cfg.g * z_rel
    return (vz + math.sqrt(disc)) / cfg.g


def wind_drift(wind_xy, t_fall: float,
               cfg: BallisticsCfg = BallisticsCfg()) -> np.ndarray:
    """落点风偏 = 风速 · 下落时间 · 拖曳系数（水平向量）。"""
    w = np.asarray(wind_xy, dtype=float)
    return w * float(t_fall) * cfg.wind_coeff


def scatter_sigma(h: float, cfg: BallisticsCfg = BallisticsCfg()) -> float:
    """经验散布半径 σ = 0.15 h（工程近似，用于给出命中概率量级）。"""
    return cfg.sigma_factor * float(h)


def impact_point(state_3d, vel_3d, wind_xy,
                 cfg: BallisticsCfg = BallisticsCfg()) -> np.ndarray:
    """由释放时的三维状态预测落点（水平二维）。

    t 由实际高度与竖直速度解出，落点 = 水平位置 + 水平速度·t + 风偏(t)。
    与"用计划高度算 t"的区别在于：掉高、抬头/低头释放都会被如实反映
    到落点偏差里。
    """
    p = np.asarray(state_3d, dtype=float)
    v = np.asarray(vel_3d, dtype=float)
    if p.size != 3 or v.size != 3:
        raise ValueError("state and velocity must be 3-D vectors")
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(v)):
        raise ValueError("non-finite release state: %r / %r"
                         % (p.tolist(), v.tolist()))
    t = fall_time_from_state(float(p[2]), float(v[2]), cfg)
    return p[:2] + v[:2] * t + wind_drift(wind_xy, t, cfg)
