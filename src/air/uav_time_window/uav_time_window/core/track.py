#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务序列 → 航路点串 → 4D 航迹（沿 ``uav_deconflict`` 的 ``Plan`` 模式）。

一条 ``RouteTrack`` 由三类匀速/悬停时空段组成：

1. **爬升转场段**：从起点地面沿斜线爬到巡航高度、同时向第一个任务点
   平移——水平分量恒等于巡航速度，因此到达第一点的时刻与
   ``window.schedule_uav`` 的解析时刻严格一致（爬升"顺路"完成，
   不额外占时间）；
2. **平飞段**：任务点之间以巡航速度在 ``alt`` 高度直线飞行；
3. **悬停段**：在每个任务点上空从 ``arrival`` 悬停到 ``departure``
   （早到等待 + 服务时长），速度为零。

所有段都是时间解析可查的（``position_at`` / ``velocity_at``），最后一
任务服务完毕后垂直下降落地。参考速度前馈在闭环里经一阶低通平滑，
模式与 ``uav_deconflict.core.sim`` 相同。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .mission import Task, Uav
from .window import UavSchedule


@dataclass
class Segment:
    """一段匀速直线运动（含零速悬停）。"""

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
class RouteTrack:
    """一架机执行任务序列的 4D 航迹。

    Parameters
    ----------
    uid
        机号，例如 ``"UAV-1"``。
    schedule
        ``window.schedule_uav`` 产出的时刻表（到达/等待/离开时刻）。
    uav
        该机的起点与速度。
    tasks
        全部任务点（含位置与服务时长）。
    alt
        巡航高度（m）。
    climb_speed
        爬升/下降率（m/s），用于最后的垂直下降段。
    """

    uid: str
    schedule: UavSchedule
    uav: Uav
    tasks: Dict[str, Task]
    alt: float = 30.0
    climb_speed: float = 2.5

    segments: List[Segment] = field(default_factory=list, init=False, repr=False)
    _starts: List[float] = field(default_factory=list, init=False, repr=False)
    planned_arrival: Dict[str, float] = field(default_factory=dict, init=False,
                                              repr=False)

    def __post_init__(self) -> None:
        if self.alt < 0.0:
            raise ValueError("巡航高度不能为负: %r" % (self.alt,))
        if self.climb_speed <= 0.0:
            raise ValueError("爬升率必须为正: %r" % (self.climb_speed,))
        self._build()

    # ------------------------------------------------------------------ 构建
    def _build(self) -> None:
        segs: List[Segment] = []
        px, py = self.uav.pos
        t = 0.0
        for stop in self.schedule.stops:
            task = self.tasks[stop.tid]
            tx, ty = task.pos
            flight = stop.arrival - t
            if flight > 1e-9:
                segs.append(Segment(t, stop.arrival,
                                    np.array([px, py, 0.0 if not segs else self.alt]),
                                    np.array([tx, ty, self.alt])))
            # 悬停：早到等待 + 服务，全程压在任务点上空
            hover_end = max(stop.departure, stop.arrival)
            segs.append(Segment(stop.arrival, hover_end,
                                np.array([tx, ty, self.alt]),
                                np.array([tx, ty, self.alt])))
            self.planned_arrival[stop.tid] = stop.arrival
            px, py = tx, ty
            t = stop.departure
        if segs:
            xe, ye = px, py
            desc = self.alt / self.climb_speed
            segs.append(Segment(t, t + desc,
                                np.array([xe, ye, self.alt]),
                                np.array([xe, ye, 0.0])))
        self.segments = [s for s in segs if s.t1 > s.t0]
        self._starts = [s.t0 for s in self.segments]

    # ------------------------------------------------------------------ 查询
    @property
    def start_time(self) -> float:
        return self.segments[0].t0 if self.segments else 0.0

    @property
    def end_time(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    @property
    def waypoints(self) -> List[Tuple[float, float]]:
        """二维航路点串（起点 + 逐任务点）。"""
        pts = [self.uav.pos]
        pts += [self.tasks[s.tid].pos for s in self.schedule.stops]
        return pts

    def segment_at(self, t: float) -> Optional[Segment]:
        if not self.segments or t < self.start_time or t > self.end_time:
            return None
        lo, hi = 0, len(self.segments) - 1
        while lo < hi:                       # 段数少，二分即可
            mid = (lo + hi + 1) // 2
            if self._starts[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return self.segments[lo]

    def position_at(self, t: float) -> Optional[np.ndarray]:
        """返回 t 时刻的 ENU 位置；未起飞或已落地返回 ``None``。"""
        seg = self.segment_at(t)
        return None if seg is None else seg.position(t)

    def velocity_at(self, t: float) -> Optional[np.ndarray]:
        seg = self.segment_at(t)
        return None if seg is None else seg.velocity

    def hold_position(self, t: float) -> np.ndarray:
        """闭环控制的参考位置：任务结束后保持在终点地面。"""
        if t <= self.start_time or not self.segments:
            x0, y0 = self.uav.pos
            return np.array([x0, y0, 0.0])
        if t >= self.end_time:
            xe, ye = self.waypoints[-1]
            return np.array([xe, ye, 0.0])
        pos = self.position_at(t)
        assert pos is not None
        return pos

    def summary(self) -> dict:
        return dict(uid=self.uid,
                    waypoints=[[round(x, 1), round(y, 1)] for x, y in self.waypoints],
                    alt=round(self.alt, 1),
                    start_time=round(self.start_time, 2),
                    end_time=round(self.end_time, 2),
                    planned_arrival={k: round(v, 2)
                                     for k, v in self.planned_arrival.items()})


def build_tracks(schedules: Dict[str, UavSchedule],
                 uavs: Dict[str, Uav], tasks: Dict[str, Task],
                 alt: float = 30.0, climb_speed: float = 2.5) -> Dict[str, RouteTrack]:
    """为每架有任务的机生成 4D 航迹（无任务的机不生成）。"""
    return {uid: RouteTrack(uid=uid, schedule=sched, uav=uavs[uid], tasks=tasks,
                            alt=alt, climb_speed=climb_speed)
            for uid, sched in schedules.items() if sched.stops}
