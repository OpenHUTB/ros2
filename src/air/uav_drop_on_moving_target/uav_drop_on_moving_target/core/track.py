#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机标称航迹：折线 + 恒速，任意时刻的位置/速度解析给出。

一行摘要：把"无人机在 t 时刻会飞到哪、朝哪个方向"变成一个纯函数，
落点解算与闭环参考都从同一条标称航迹取值，避免两处各算一遍。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class PolylineTrack:
    """折线航迹（水平面内），从 points[0] 起以 speed 恒速飞行。"""

    points: Tuple[Tuple[float, float], ...]
    speed: float
    #: 起步加速时间（s）：从静止匀加速到 speed，之后匀速。爬升结束时若直接
    #: 切到匀速前馈，机体会掉高、参考点也会瞬间跑掉，所以把加速段写进航迹
    #: 模型本身——解算与真飞因此共用同一条时间参数化
    ramp_s: float = 3.0

    def __post_init__(self):
        if not np.isfinite(self.speed) or self.speed <= 0.0:
            raise ValueError("track speed must be positive and finite")
        if len(self.points) < 2:
            raise ValueError("track needs >= 2 points")
        if not np.isfinite(self.ramp_s) or self.ramp_s < 0.0:
            raise ValueError("ramp_s must be non-negative and finite")
        # 提前触发"零长度"检查：构造时就把不可用的航迹拒掉，而不是等到
        # 第一次求解才发现
        self.segs
        for x, y in self.points:
            if not np.isfinite(x) or not np.isfinite(y):
                raise ValueError("non-finite track point: %r" % ((x, y),))

    @property
    def segs(self) -> List[Tuple[np.ndarray, np.ndarray, float, np.ndarray]]:
        """[(a, b, 长度, 单位方向), ...]；长度为 0 的段会被跳过。"""
        out = []
        for i in range(len(self.points) - 1):
            a = np.asarray(self.points[i], dtype=float)
            b = np.asarray(self.points[i + 1], dtype=float)
            d = b - a
            n = float(np.linalg.norm(d))
            if n > 1e-9:
                out.append((a, b, n, d / n))
        if not out:
            raise ValueError("track has zero total length")
        return out

    @property
    def length_m(self) -> float:
        return float(sum(s[2] for s in self.segs))

    @property
    def duration_s(self) -> float:
        """飞完全程的时间（含起步加速的额外 0.5·ramp）。"""
        return self.length_m / self.speed + 0.5 * self.ramp_s

    def pos_vel_at_s(self, s: float) -> Tuple[np.ndarray, np.ndarray]:
        """弧长 s 处的位置与速度（超出末端停在终点，速度置零）。"""
        s = max(0.0, float(s))
        for a, b, n, u in self.segs:
            if s <= n + 1e-12:
                return a + u * s, u * self.speed
            s -= n
        end = np.asarray(self.segs[-1][1], dtype=float)
        return end, np.zeros(2)

    def pos_vel(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """时刻 t 的位置与速度（含起步加速段）。"""
        if not np.isfinite(t):
            raise ValueError("non-finite time: %r" % t)
        t = max(0.0, float(t))
        if self.ramp_s > 0.0 and t < self.ramp_s:
            a = self.speed / self.ramp_s
            p, _ = self.pos_vel_at_s(0.5 * a * t * t)
            return p, self.segs[0][3] * (a * t)
        return self.pos_vel_at_s(self.speed * (t - 0.5 * self.ramp_s))


def straight_track(start_xy, end_xy, speed: float) -> PolylineTrack:
    """单段直线航迹（算例里最常用）。"""
    return PolylineTrack((tuple(start_xy), tuple(end_xy)), speed)
