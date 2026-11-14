#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地面接收车：折线路径 + 分段恒速 + 可在指定航点停留。

一行摘要：投放要打的不是"车现在在哪"而是"货落地时车在哪"，所以车辆
必须能回答任意未来时刻的位置——包括中途停车再起步的情形。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class Vehicle:
    """地面车辆：waypoints 折线，speed 恒速，dwell_s 指定航点停留。

    dwell_s 的每一项是 (航点序号, 停留秒数)：序号 0 是起点（一般不停），
    1 是第一个拐点，依此类推；停留期间速度为零、位置钉在该航点。
    """

    name: str
    waypoints: Tuple[Tuple[float, float], ...]
    speed: float
    dwell_s: Tuple[Tuple[int, float], ...] = ()

    def __post_init__(self):
        if not np.isfinite(self.speed) or self.speed <= 0.0:
            raise ValueError("vehicle speed must be positive and finite, got %r"
                             % self.speed)
        if len(self.waypoints) < 2:
            raise ValueError("vehicle path needs >= 2 waypoints")
        for x, y in self.waypoints:
            if not np.isfinite(x) or not np.isfinite(y):
                raise ValueError("non-finite waypoint: %r" % ((x, y),))
        for idx, dur in self.dwell_s:
            if not (0 <= int(idx) < len(self.waypoints)):
                raise ValueError("dwell waypoint index out of range: %r" % (idx,))
            if not np.isfinite(dur) or dur < 0.0:
                raise ValueError("dwell must be non-negative and finite")

    # ------------------------------------------------------------------ #
    @property
    def segs(self) -> List[Tuple[np.ndarray, np.ndarray, float, np.ndarray]]:
        """[(a, b, 长度, 单位方向), ...]；零长度段被跳过。"""
        out = []
        for i in range(len(self.waypoints) - 1):
            a = np.asarray(self.waypoints[i], dtype=float)
            b = np.asarray(self.waypoints[i + 1], dtype=float)
            d = b - a
            n = float(np.linalg.norm(d))
            if n > 1e-9:
                out.append((a, b, n, d / n))
        if not out:
            raise ValueError("vehicle path has zero total length")
        return out

    @property
    def total_len_m(self) -> float:
        return float(sum(s[2] for s in self.segs))

    @property
    def total_dwell_s(self) -> float:
        return float(sum(d for _i, d in self.dwell_s))

    @property
    def duration_s(self) -> float:
        """走完全程（含停留）的时间；之后停在终点。"""
        return self.total_len_m / self.speed + self.total_dwell_s

    def dwell_at(self, waypoint_index: int) -> float:
        for idx, dur in self.dwell_s:
            if int(idx) == int(waypoint_index):
                return float(dur)
        return 0.0

    def arrival_time(self, waypoint_index: int) -> float:
        """到达第 waypoint_index 个航点的时刻（不含该点停留）。"""
        t = 0.0
        for i, (_a, _b, n, _u) in enumerate(self.segs):
            t += n / self.speed
            if i + 1 == int(waypoint_index):
                return t
            t += self.dwell_at(i + 1)
        raise ValueError("waypoint index out of range: %r" % waypoint_index)

    # ------------------------------------------------------------------ #
    def pos_vel(self, t: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """t 时刻的 (位置2, 速度2, 已行驶弧长)。"""
        if not np.isfinite(t):
            raise ValueError("non-finite time: %r" % t)
        t = max(0.0, float(t))
        clock = 0.0
        travelled = 0.0
        for i, (a, b, n, u) in enumerate(self.segs):
            leg = n / self.speed
            if t <= clock + leg + 1e-12:
                dt = t - clock
                return a + u * (self.speed * dt), u * self.speed, travelled + self.speed * dt
            clock += leg
            travelled += n
            dwell = self.dwell_at(i + 1)
            if dwell > 0.0 and t <= clock + dwell + 1e-12:
                return b.copy(), np.zeros(2), travelled
            clock += dwell
        end = np.asarray(self.segs[-1][1], dtype=float)
        return end, np.zeros(2), self.total_len_m

    def pos_at(self, t: float) -> np.ndarray:
        return self.pos_vel(t)[0]

    def speed_at(self, t: float) -> float:
        return float(np.linalg.norm(self.pos_vel(t)[1]))

    def prediction_error(self, t_release: float, t_fall: float) -> float:
        """投放后车辆又走了多远：|veh(t_r + t_fall) − veh(t_r)|。

        这一项就是"瞄准车辆当前位置"必然产生的落点偏差，与无人机的飞行
        速度、高度都无关——只要目标在动，它就不会是零。
        """
        if t_fall < 0.0:
            raise ValueError("t_fall must be non-negative")
        p_now = self.pos_at(t_release)
        p_then = self.pos_at(t_release + t_fall)
        return float(np.linalg.norm(p_then - p_now))
