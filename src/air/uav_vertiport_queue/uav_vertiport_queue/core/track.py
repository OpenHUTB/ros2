#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""进离场航迹生成：三段匀速直线模式（与参考实现 core/plan.py 同一思路）。

所有长度单位 m、时间单位 s、坐标 ENU（x 东、y 北、z 上）。

离场（departure）三段，起点是起飞位地面：

1. 垂直爬升 0 → 过渡高度 60 m；
2. 60 m 转巡航：从起飞位上方匀速直线爬升到离场航路点，并同时爬到巡航高度；
3. 到达航路点后任务结束（离开起降场空域）。

进场（arrival）三段，终点是降落位地面：

1. 巡航高度（默认 100 m）下降到 60 m 过渡高度（在进场定位点正上方）；
2. 60 m 进近直线：从定位点匀速直线飞到降落位上方（保持 60 m）；
3. 垂直下降 60 m → 0（接地）。

三段都是匀速直线运动，任意时刻位姿可解析求解，因此闭环前的计划指标
（理论接地/离地时刻、段长）可复现、可单元测试。
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .queue import ARRIVAL, DEPARTURE

#: 进离场过渡高度（m）——题目规定的 60 m
GATE_ALT = 60.0


@dataclass
class Segment:
    """一段匀速直线运动（时空）。"""

    t0: float
    t1: float
    p0: np.ndarray
    p1: np.ndarray

    @property
    def duration(self) -> float:
        return max(0.0, self.t1 - self.t0)

    @property
    def velocity(self) -> np.ndarray:
        d = self.t1 - self.t0
        if d <= 1e-12:
            return np.zeros(3)
        return (self.p1 - self.p0) / d

    def position(self, t: float) -> np.ndarray:
        d = self.t1 - self.t0
        if d <= 1e-12:
            return self.p1.copy()
        u = min(1.0, max(0.0, (t - self.t0) / d))
        return self.p0 + (self.p1 - self.p0) * u


@dataclass
class Track:
    """一架无人机在起降场空域内的三段航迹。

    Parameters
    ----------
    uid
        机号。
    kind
        ``"departure"``（离场）或 ``"arrival"``（进场）。
    op_time
        关键运行时刻：离场 = 计划离地时刻（= 航迹起点），进场 = 计划接地
        时刻（= 航迹终点）。排队模块输出的 ``t_scheduled`` 就填这里。
    """

    uid: str
    kind: str
    op_time: float
    segments: List[Segment] = field(default_factory=list)
    label: str = ""

    _starts: List[float] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.kind not in (DEPARTURE, ARRIVAL):
            raise ValueError("kind 必须是 departure/arrival: %r" % (self.kind,))
        if not self.segments:
            raise ValueError("航迹至少需要一段")
        for s in self.segments:
            if s.t1 < s.t0 - 1e-9:
                raise ValueError("段时间倒挂: %r" % (s,))
        self._starts = [s.t0 for s in self.segments]

    # ------------------------------------------------------------------ 查询
    @property
    def start_time(self) -> float:
        return self.segments[0].t0

    @property
    def end_time(self) -> float:
        return self.segments[-1].t1

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    def is_active(self, t: float) -> bool:
        return self.start_time - 1e-9 <= t <= self.end_time + 1e-9

    def segment_at(self, t: float) -> Optional[Segment]:
        if t < self.start_time - 1e-9 or t > self.end_time + 1e-9:
            return None
        i = bisect.bisect_right(self._starts, t) - 1
        i = min(max(i, 0), len(self.segments) - 1)
        return self.segments[i]

    def position_at(self, t: float) -> Optional[np.ndarray]:
        seg = self.segment_at(t)
        return None if seg is None else seg.position(t)

    def velocity_at(self, t: float) -> Optional[np.ndarray]:
        seg = self.segment_at(t)
        return None if seg is None else seg.velocity

    def hold_position(self, t: float) -> np.ndarray:
        """闭环控制参考位置：未开始保持在起点，结束后保持在终点。"""
        if t <= self.start_time:
            return self.segments[0].p0.copy()
        if t >= self.end_time:
            return self.segments[-1].p1.copy()
        pos = self.position_at(t)
        assert pos is not None
        return pos

    def summary(self) -> dict:
        return dict(uid=self.uid, kind=self.kind, label=self.label,
                    op_time=round(self.op_time, 2),
                    start_time=round(self.start_time, 2),
                    end_time=round(self.end_time, 2),
                    duration=round(self.duration, 2))


# --------------------------------------------------------------------------- #
# 生成器
# --------------------------------------------------------------------------- #
def departure_track(uid: str, pad_xy: Sequence[float],
                    exit_xy: Sequence[float], t_takeoff: float,
                    cruise_alt: float = 100.0, gate_alt: float = GATE_ALT,
                    speed: float = 8.0, climb_speed: float = 2.5,
                    label: str = "") -> Track:
    """离场航迹：起飞位 → 垂直爬升到 60 m → 边爬边飞到离场点 → 巡航高度。"""
    if cruise_alt < gate_alt:
        raise ValueError("巡航高度必须不低于过渡高度 %.0f m" % gate_alt)
    px, py = float(pad_xy[0]), float(pad_xy[1])
    ex, ey = float(exit_xy[0]), float(exit_xy[1])
    t = float(t_takeoff)
    segs = [Segment(t, t + gate_alt / climb_speed,
                    np.array([px, py, 0.0]),
                    np.array([px, py, gate_alt]))]
    t += segs[-1].duration
    segs.append(Segment(t, t + gate_alt / climb_speed,
                        np.array([px, py, gate_alt]),
                        np.array([px, py, cruise_alt])))
    t += segs[-1].duration
    dist = float(np.hypot(ex - px, ey - py))
    segs.append(Segment(t, t + dist / speed,
                        np.array([px, py, cruise_alt]),
                        np.array([ex, ey, cruise_alt])))
    return Track(uid=uid, kind=DEPARTURE, op_time=float(t_takeoff),
                 segments=segs, label=label)


def arrival_track(uid: str, fix_xy: Sequence[float],
                  pad_xy: Sequence[float], t_touchdown: float,
                  cruise_alt: float = 100.0, gate_alt: float = GATE_ALT,
                  speed: float = 8.0, descent_speed: float = 2.0,
                  label: str = "") -> Track:
    """进场航迹：进场点从巡航高度降到 60 m → 60 m 进近直线 → 垂直降落。

    ``t_touchdown`` 是计划接地时刻，三段时间全部从它倒推。
    """
    if cruise_alt < gate_alt:
        raise ValueError("巡航高度必须不低于过渡高度 %.0f m" % gate_alt)
    fx, fy = float(fix_xy[0]), float(fix_xy[1])
    px, py = float(pad_xy[0]), float(pad_xy[1])
    t_end = float(t_touchdown)
    # 段3：60 m → 0 垂直降落
    seg3 = Segment(t_end - gate_alt / descent_speed, t_end,
                   np.array([px, py, gate_alt]), np.array([px, py, 0.0]))
    # 段2：定位点 → 降落位上方，60 m 进近直线
    dist = float(np.hypot(px - fx, py - fy))
    seg2 = Segment(seg3.t0 - dist / speed, seg3.t0,
                   np.array([fx, fy, gate_alt]), np.array([px, py, gate_alt]))
    # 段1：定位点正上方巡航高度 → 60 m
    seg1 = Segment(seg2.t0 - (cruise_alt - gate_alt) / descent_speed, seg2.t0,
                   np.array([fx, fy, cruise_alt]),
                   np.array([fx, fy, gate_alt]))
    return Track(uid=uid, kind=ARRIVAL, op_time=t_end,
                 segments=[seg1, seg2, seg3], label=label)
