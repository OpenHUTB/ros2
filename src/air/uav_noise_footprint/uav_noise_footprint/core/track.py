#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞行剖面：二维航路 + 巡航高度 + 爬升角 → 分段线性时空轨迹。

所有长度单位 m、时间单位 s、坐标 ENU（x 东、y 北、z 上）。

一条剖面的地面投影就是给定的二维折线 ``path``，但**爬升/下降段是斜的**：
起飞时以爬升角 ``climb_angle``（相对水平面）沿第一段航向爬到巡航高度，
降落时沿最后一段航向的反方向下降。因此爬升段的水平投影长度是
``alt / tan(climb_angle)``，要求首末航段长度足够容纳它。

三段都是匀速直线运动（沿轨迹速度 = ``speed``），可以解析求解任意时刻的
位置与速度——这是噪声积分采样、闭环前馈与单元测试可复现的前提。
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


def _unit(a, b) -> Tuple[float, float]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n <= 1e-9:
        raise ValueError("航路段长度为零")
    return dx / n, dy / n


@dataclass
class Flight:
    """一架无人机的飞行剖面。

    Parameters
    ----------
    uid
        机号，例如 ``"UAV-1"``。
    path
        二维航路点 ``[(x, y), ...]``，至少两个点；首段需容纳爬升水平投影。
    alt
        巡航高度（m，AGL 近似为海拔）。
    t0
        起飞时刻（s）。
    speed
        沿轨迹速度（m/s），爬升/巡航/下降通用。
    climb_angle
        起降段相对水平面的爬升角（deg）。
    label
        人类可读的任务名，仅用于报告与图例。
    """

    uid: str
    path: Sequence[Tuple[float, float]]
    alt: float = 30.0
    t0: float = 0.0
    speed: float = 8.0
    climb_angle: float = 8.0
    label: str = ""

    segments: List[Segment] = field(default_factory=list, init=False, repr=False)
    _starts: List[float] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        self.path = [(float(x), float(y)) for x, y in self.path]
        if len(self.path) < 2:
            raise ValueError("航路至少需要两个点: %s" % (self.path,))
        if self.alt <= 0.0:
            raise ValueError("巡航高度必须为正: %r" % (self.alt,))
        if self.speed <= 0.0:
            raise ValueError("速度必须为正: %r" % (self.speed,))
        if not (0.0 < self.climb_angle < 90.0):
            raise ValueError("爬升角必须在 (0, 90) deg: %r" % (self.climb_angle,))
        self._build()

    # ------------------------------------------------------------------ 几何
    @property
    def climb_run(self) -> float:
        """爬升段的水平投影长度 alt / tan(angle)。"""
        return self.alt / math.tan(math.radians(self.climb_angle))

    def _build(self) -> None:
        ang = math.radians(self.climb_angle)
        sin_a, cos_a = math.sin(ang), math.cos(ang)
        run = self.climb_run

        first, last = self.path[0], self.path[-1]
        l_first = math.hypot(*(self.path[1][k] - first[k] for k in (0, 1)))
        l_last = math.hypot(*(last[k] - self.path[-2][k] for k in (0, 1)))
        if run > l_first + 1e-6:
            raise ValueError(
                "首航段 %.1f m 容不下 %.1f m 爬升投影（增大爬升角或缩短航段）"
                % (l_first, run))
        if run > l_last + 1e-6:
            raise ValueError(
                "末航段 %.1f m 容不下 %.1f m 下降投影" % (l_last, run))
        if 2.0 * run >= self.path_length_ground():
            raise ValueError(
                "爬升+下降水平投影合计 %.1f m 吃满整条 %.1f m 航路，"
                "巡航段退化为负" % (2.0 * run, self.path_length_ground()))

        ux0, uy0 = _unit(first, self.path[1])
        ulx, uly = _unit(self.path[-2], last)

        # 折线上的巡航起点/终点（2D）：爬升吃掉首段开头，下降吃掉末段结尾
        cruise_pts: List[Tuple[float, float]] = [
            (first[0] + ux0 * run, first[1] + uy0 * run)]
        for pt in self.path[1:-1]:
            cruise_pts.append(pt)
        cruise_pts.append((last[0] - ulx * run, last[1] - uly * run))

        segs: List[Segment] = []
        t = float(self.t0)
        segs.append(Segment(t, t + run / (cos_a * self.speed),
                            np.array([first[0], first[1], 0.0]),
                            np.array([cruise_pts[0][0], cruise_pts[0][1], self.alt])))
        t = segs[-1].t1
        for (xa, ya), (xb, yb) in zip(cruise_pts[:-1], cruise_pts[1:]):
            dist = math.hypot(xb - xa, yb - ya)
            if dist <= 1e-9:
                continue
            segs.append(Segment(t, t + dist / self.speed,
                                np.array([xa, ya, self.alt]),
                                np.array([xb, yb, self.alt])))
            t += dist / self.speed
        segs.append(Segment(t, t + run / (cos_a * self.speed),
                            np.array([cruise_pts[-1][0], cruise_pts[-1][1], self.alt]),
                            np.array([last[0], last[1], 0.0])))
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
    def copy_with(self, **kwargs) -> "Flight":
        data = dict(uid=self.uid, path=list(self.path), alt=self.alt,
                    t0=self.t0, speed=self.speed,
                    climb_angle=self.climb_angle, label=self.label)
        data.update(kwargs)
        return Flight(**data)

    def with_altitude(self, alt: float) -> "Flight":
        return self.copy_with(alt=float(alt))

    def with_climb_angle(self, angle: float) -> "Flight":
        return self.copy_with(climb_angle=float(angle))

    def with_lateral_offset(self, offset: float, samples: int = 12) -> "Flight":
        """把**巡航段**平滑横移 ``offset``（正值沿航向左侧），起降点不动。

        横移量沿巡航弧长按余弦台阶分布——巡航起点/终点处为零、中段达到
        全量，因此爬升→巡航→下降的航向衔接处不会出现阶跃激励姿态回路。

        实现：先按当前几何求出巡航段首末端（爬升/下降的水平投影），只在
        巡航段内部插入横移后的航路点，然后整体重建剖面——``_build`` 会按
        新折线重新推导爬升/下降段。
        """
        if abs(offset) < 1e-9 or len(self.path) < 2:
            return self.copy_with()
        run = self.climb_run
        first, last = self.path[0], self.path[-1]
        ux0, uy0 = _unit(first, self.path[1])
        ulx, uly = _unit(self.path[-2], last)
        c0 = np.array([first[0] + ux0 * run, first[1] + uy0 * run])
        cn = np.array([last[0] - ulx * run, last[1] - uly * run])

        seg_vec = cn - c0
        seg_len = float(np.linalg.norm(seg_vec))
        if seg_len <= 1e-9:
            return self.copy_with()
        u = seg_vec / seg_len
        n = np.array([-u[1], u[0]])

        def taper(s: float) -> float:
            """0→1→0 的余弦台阶：两端 0、斜率 0，中段（±35%）全量。"""
            m = min(s / 0.35, (1.0 - s) / 0.35, 1.0)
            return 0.5 * (1.0 - math.cos(math.pi * max(m, 0.0)))

        new_path: List[Tuple[float, float]] = [tuple(first), tuple(c0)]
        for k in range(1, samples):
            s = k / float(samples)
            w = taper(s)
            p = c0 + seg_vec * s + n * (offset * w)
            new_path.append((float(p[0]), float(p[1])))
        new_path.append(tuple(cn))
        new_path.append(tuple(last))
        return self.copy_with(path=new_path)

    # ------------------------------------------------------------------ 指标
    def path_length_3d(self) -> float:
        return float(sum(s.length() for s in self.segments))

    def path_length_ground(self) -> float:
        return float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(self.path[:-1], self.path[1:])))

    def summary(self) -> dict:
        return dict(uid=self.uid, label=self.label,
                    t0=round(self.t0, 2), alt=round(self.alt, 2),
                    climb_angle=round(self.climb_angle, 1),
                    duration=round(self.duration, 2),
                    end_time=round(self.end_time, 2),
                    path_ground=round(self.path_length_ground(), 1),
                    path_3d=round(self.path_length_3d(), 1))

    def to_dict(self) -> dict:
        """完整可重建的字典（写结果 JSON 用）。"""
        return dict(uid=self.uid, label=self.label,
                    path=[[x, y] for x, y in self.path],
                    alt=self.alt, t0=self.t0, speed=self.speed,
                    climb_angle=self.climb_angle)

    @classmethod
    def from_dict(cls, d: dict) -> "Flight":
        return cls(d["uid"], [tuple(p) for p in d["path"]], alt=d["alt"],
                   t0=d["t0"], speed=d["speed"], climb_angle=d["climb_angle"],
                   label=d.get("label", ""))
