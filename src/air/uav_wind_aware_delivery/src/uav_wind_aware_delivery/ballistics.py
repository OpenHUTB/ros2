# -*- coding: utf-8 -*-
"""物资自由落体弹道：用于生成训练标签和评估基线。

与 MuJoCo 中施加的风阻力保持一致：
    a_xy = k_drag * (wind_xy - v_xy) / m
    a_z  = -g - 0.02 * v_z / m
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class DropConfig:
    gravity: float = 9.81
    drag_k: float = 0.32
    vertical_drag: float = 0.02
    mass: float = 0.20
    dt: float = 0.002


def simulate_drop(h: float, vx: float, vy: float, vz: float,
                  wind: np.ndarray, cfg: DropConfig | None = None,
                  return_time: bool = False):
    """从高度 h、初速度 (vx,vy,vz) 释放，返回落点 (x,y)（相对释放点）。"""
    c = cfg or DropConfig()
    wind = np.asarray(wind, dtype=float)
    p = np.array([0.0, 0.0, float(h)])
    v = np.array([float(vx), float(vy), float(vz)])
    t = 0.0
    while p[2] > 0.0 and t < 20.0:
        a = np.array([
            c.drag_k * (wind[0] - v[0]) / c.mass,
            c.drag_k * (wind[1] - v[1]) / c.mass,
            -c.gravity - c.vertical_drag * v[2] / c.mass,
        ])
        v += a * c.dt
        p += v * c.dt
        t += c.dt
    if return_time:
        return p[:2].copy(), t
    return p[:2].copy()


def release_point(target_xy: np.ndarray, h: float, wind: np.ndarray,
                  v0: np.ndarray | None = None, cfg: DropConfig | None = None,
                  iters: int = 4) -> np.ndarray:
    """迭代求释放点：目标点 - 预测落点偏移。v0 为无人机释放时水平速度。"""
    target_xy = np.asarray(target_xy, dtype=float)
    v0 = np.zeros(2) if v0 is None else np.asarray(v0, dtype=float)
    rel = target_xy.copy()
    for _ in range(iters):
        off = simulate_drop(h, v0[0], v0[1], 0.0, wind, cfg)
        rel = target_xy - off
    return rel
