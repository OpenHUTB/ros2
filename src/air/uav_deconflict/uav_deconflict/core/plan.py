#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""4D 航迹：起飞时刻 + 二维航路 + 巡航高度 → 分段线性时空轨迹。

所有长度单位 m、时间单位 s、坐标 ENU（x 东、y 北、z 上）。

一条航迹由三部分组成：

1. 在起点垂直爬升到巡航高度（爬升率 ``climb_speed``）
2. 沿二维折线以 ``speed`` 平飞
3. 在终点垂直下降

三段都是匀速直线运动，因此可以解析求解任意时刻的位姿，也可以解析计算
两条航迹之间的最近接近点（CPA），不需要数值采样——这一点是冲突检测
可复现、可写单元测试的前提。
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


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

    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))


@dataclass
class Plan:
    """一架无人机的 4D 航迹。

    Parameters
    ----------
    uid
        机号，例如 ``"UAV-1"``。
    path
        二维航路点 ``[(x, y), ...]``，至少两个点；起终点自动落到地面高度 0。
    alt
        巡航高度（m，AGL 近似为海拔）。
    t0
        起飞时刻（s）。
    speed
        平飞速度（m/s）。
    climb_speed
        爬升/下降率（m/s）。
    priority
        任务优先级，数值越大越不应被延误（例如医疗急救 = 10，普通配送 = 0）。
    label
        人类可读的任务名，仅用于报告与图例。
    """

    uid: str
    path: Sequence[Tuple[float, float]]
    alt: float = 60.0
    t0: float = 0.0
    speed: float = 8.0
    climb_speed: float = 2.5
    priority: int = 0
    label: str = ""

    segments: List[Segment] = field(default_factory=list, init=False, repr=False)
    _starts: List[float] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        self.path = [(float(x), float(y)) for x, y in self.path]
        if len(self.path) < 2:
            raise ValueError("航路至少需要两个点: %s" % (self.path,))
        if self.alt < 0.0:
            raise ValueError("巡航高度不能为负: %r" % (self.alt,))
        if self.speed <= 0.0 or self.climb_speed <= 0.0:
            raise ValueError("速度必须为正: speed=%r climb_speed=%r"
                             % (self.speed, self.climb_speed))
        self._build()

    # ------------------------------------------------------------------ 构建
    def _build(self) -> None:
        segs: List[Segment] = []
        t = float(self.t0)
        x0, y0 = self.path[0]

        if self.alt > 0.0:
            climb = self.alt / self.climb_speed
            segs.append(Segment(t, t + climb,
                                np.array([x0, y0, 0.0]),
                                np.array([x0, y0, self.alt])))
            t += climb

        for (xa, ya), (xb, yb) in zip(self.path[:-1], self.path[1:]):
            dist = math.hypot(xb - xa, yb - ya)
            if dist <= 1e-9:
                continue
            dt = dist / self.speed
            segs.append(Segment(t, t + dt,
                                np.array([xa, ya, self.alt]),
                                np.array([xb, yb, self.alt])))
            t += dt

        if self.alt > 0.0:
            xe, ye = self.path[-1]
            desc = self.alt / self.climb_speed
            segs.append(Segment(t, t + desc,
                                np.array([xe, ye, self.alt]),
                                np.array([xe, ye, 0.0])))
            t += desc

        self.segments = segs
        self._starts = [s.t0 for s in segs]

    # ------------------------------------------------------------------ 查询
    @property
    def start_time(self) -> float:
        return self.segments[0].t0 if self.segments else float(self.t0)

    @property
    def end_time(self) -> float:
        return self.segments[-1].t1 if self.segments else float(self.t0)

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    def is_active(self, t: float) -> bool:
        return self.start_time - 1e-9 <= t <= self.end_time + 1e-9

    def segment_at(self, t: float) -> Optional[Segment]:
        if not self.segments or t < self.start_time or t > self.end_time:
            return None
        i = bisect.bisect_right(self._starts, t) - 1
        i = min(max(i, 0), len(self.segments) - 1)
        return self.segments[i]

    def position_at(self, t: float) -> Optional[np.ndarray]:
        """返回 t 时刻的 ENU 位置；未起飞或已落地时返回 ``None``。"""
        seg = self.segment_at(t)
        return None if seg is None else seg.position(t)

    def velocity_at(self, t: float) -> Optional[np.ndarray]:
        seg = self.segment_at(t)
        return None if seg is None else seg.velocity

    def hold_position(self, t: float) -> np.ndarray:
        """用于闭环控制的参考位置：任务结束后保持在终点（已落地）。"""
        if t <= self.start_time:
            x0, y0 = self.path[0]
            return np.array([x0, y0, 0.0])
        if t >= self.end_time:
            xe, ye = self.path[-1]
            return np.array([xe, ye, 0.0])
        pos = self.position_at(t)
        assert pos is not None
        return pos

    # ------------------------------------------------------------------ 变体
    def copy_with(self, **kwargs) -> "Plan":
        data = dict(uid=self.uid, path=list(self.path), alt=self.alt, t0=self.t0,
                    speed=self.speed, climb_speed=self.climb_speed,
                    priority=self.priority, label=self.label)
        data.update(kwargs)
        return Plan(**data)

    def with_ground_delay(self, dt: float) -> "Plan":
        """地面等待：起飞时刻整体后移，航路与速度不变。"""
        return self.copy_with(t0=self.t0 + float(dt))

    def with_altitude(self, alt: float) -> "Plan":
        return self.copy_with(alt=float(alt))

    def with_lateral_offset(self, offset: float,
                            segment_index: Optional[int] = None,
                            samples: int = 8) -> "Plan":
        """在指定平飞段上叠加平滑横向偏移（正值为航向左侧）。

        横向偏移是低空冲突解脱里最常用的几何手段：绕开冲突区后回到原航路，
        对到达时刻几乎没有影响。偏移量沿航段按 ``(1 - cos 2πs)/2`` 分布——
        两端为零、中点为 ``offset``，且两端斜率为零，因此不会出现航向阶跃。

        这一点在闭环里很关键：早期版本只插一个折点，航向在一帧内突变，
        几何控制器姿态回路饱和后整机翻转坠落（metro 场景实测跟踪误差发散
        到 2×10^5 m）。平滑后同一场景的最大跟踪误差回到 5 m 量级。
        """
        if abs(offset) < 1e-9 or len(self.path) < 2:
            return self.copy_with()
        if segment_index is None:
            # 默认取最长的一段（通常是主巡航段）
            lens = [math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(self.path[:-1], self.path[1:])]
            segment_index = int(np.argmax(lens))
        segment_index = min(max(segment_index, 0), len(self.path) - 2)
        (xa, ya), (xb, yb) = self.path[segment_index], self.path[segment_index + 1]
        dx, dy = xb - xa, yb - ya
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return self.copy_with()
        ux, uy = dx / length, dy / length
        samples = max(2, int(samples))

        new_path = list(self.path[:segment_index + 1])
        for k in range(1, samples):
            s = k / float(samples)
            w = 0.5 * (1.0 - math.cos(2.0 * math.pi * s))
            # 沿航段推进 L·s，法向（左）偏移 offset·w
            px = xa + ux * length * s - uy * offset * w
            py = ya + uy * length * s + ux * offset * w
            new_path.append((px, py))
        new_path.extend(self.path[segment_index + 1:])
        return self.copy_with(path=new_path)

    # ------------------------------------------------------------------ 指标
    def path_length_xy(self) -> float:
        return float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(self.path[:-1], self.path[1:])))

    def summary(self) -> dict:
        return dict(uid=self.uid, label=self.label,
                    t0=round(self.t0, 2), alt=round(self.alt, 2),
                    duration=round(self.duration, 2),
                    end_time=round(self.end_time, 2),
                    path_xy=round(self.path_length_xy(), 1),
                    priority=self.priority)


def fleet_summary(plans: Sequence[Plan]) -> List[dict]:
    return [p.summary() for p in plans]
