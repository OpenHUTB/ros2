#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三个可复现的相遇场景：headon（对头）、cross（十字交叉）、overtake（追越）。

所有场景都是"两机已近距离相遇"的在线局面：每架无人机沿自己的计划航路
（起飞爬升 → 直线平飞 → 下降）直飞，冲突来自航路几何而不是人为摆位。
每个场景 4 架，基线（无规则直飞）与规则启用共用同一批计划。
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class FlightPlan:
    """一架无人机的计划：二维航路（直线）+ 巡航高度 + 巡航速度。

    运动分三段：起点垂直爬升到 ``alt`` → 沿航路匀速平飞 → 终点垂直下降。
    与参考实现 ``Plan`` 同构，但只保留在线决策需要的查询接口。
    """

    uid: str
    start: Tuple[float, float]
    end: Tuple[float, float]
    alt: float = 15.0            # m
    speed: float = 8.0           # m/s
    climb_speed: float = 2.5     # m/s
    priority: int = 0
    label: str = ""

    _t_cruise0: float = field(default=0.0, init=False, repr=False)
    _t_cruise1: float = field(default=0.0, init=False, repr=False)
    _t_end: float = field(default=0.0, init=False, repr=False)

    def __post_init__(self) -> None:
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        if math.hypot(dx, dy) <= 1e-9:
            raise ValueError("%s 航路起终点重合" % self.uid)
        if self.alt < 0.0 or self.speed <= 0.0 or self.climb_speed <= 0.0:
            raise ValueError("%s 参数非法: alt=%r speed=%r" % (self.uid, self.alt,
                                                              self.speed))
        self._cruise_len = math.hypot(dx, dy)
        climb = self.alt / self.climb_speed
        self._t_cruise0 = climb
        self._t_cruise1 = climb + self._cruise_len / self.speed
        self._t_end = self._t_cruise1 + climb

    # ---------------------------------------------------------------- 查询
    @property
    def cruise_heading(self) -> float:
        return math.atan2(self.end[1] - self.start[1],
                          self.end[0] - self.start[0])

    @property
    def cruise_start(self) -> np.ndarray:
        return np.array([self.start[0], self.start[1], self.alt])

    @property
    def cruise_end(self) -> np.ndarray:
        return np.array([self.end[0], self.end[1], self.alt])

    @property
    def cruise_t0(self) -> float:
        return self._t_cruise0

    @property
    def cruise_t1(self) -> float:
        return self._t_cruise1

    @property
    def end_time(self) -> float:
        return self._t_end

    def in_cruise(self, t: float) -> bool:
        return self.cruise_t0 - 1e-9 <= t <= self.cruise_t1 + 1e-9

    def cruise_progress(self, t: float) -> float:
        """巡航段已飞距离（m，截断到 [0, L]）。"""
        return min(max((t - self.cruise_t0) * self.speed, 0.0), self._cruise_len)

    def position_at(self, t: float) -> np.ndarray:
        """计划位置（爬升→平飞→下降三段匀速）。"""
        u = np.array([self.start[0], self.start[1], 0.0])
        e = np.array([self.end[0], self.end[1], 0.0])
        climb = self.alt / self.climb_speed
        if t <= 0.0:
            return u
        if t < climb:
            return u + np.array([0.0, 0.0, self.alt * t / climb])
        if t < self.cruise_t1:
            s = self.cruise_progress(t)
            L = max(self._cruise_len, 1e-9)
            return np.array([self.start[0] + (self.end[0] - self.start[0]) * s / L,
                             self.start[1] + (self.end[1] - self.start[1]) * s / L,
                             self.alt])
        if t < self._t_end:
            return e + np.array([0.0, 0.0, self.alt * (self._t_end - t) / climb])
        return e

    def velocity_at(self, t: float) -> np.ndarray:
        climb = self.alt / self.climb_speed
        if t < climb:
            return np.array([0.0, 0.0, self.climb_speed])
        if t < self.cruise_t1:
            h = self.cruise_heading
            return np.array([self.speed * math.cos(h), self.speed * math.sin(h),
                             0.0])
        return np.array([0.0, 0.0, -self.climb_speed])

    def cruise_unit(self) -> np.ndarray:
        h = self.cruise_heading
        return np.array([math.cos(h), math.sin(h)])

    def cruise_left_normal(self) -> np.ndarray:
        h = self.cruise_heading
        return np.array([-math.sin(h), math.cos(h)])

    def cross_track(self, xy: np.ndarray) -> float:
        """点到巡航航线的有符号横向偏移（沿航向左侧为正）。"""
        p0 = self.cruise_start[:2]
        n = self.cruise_left_normal()
        return float(np.dot(np.asarray(xy[:2], dtype=float) - p0, n))

    def along_track(self, xy: np.ndarray) -> float:
        u = self.cruise_unit()
        p0 = self.cruise_start[:2]
        return float(np.dot(np.asarray(xy[:2], dtype=float) - p0, u))

    def summary(self) -> dict:
        return dict(uid=self.uid, label=self.label,
                    start=[round(self.start[0], 1), round(self.start[1], 1)],
                    end=[round(self.end[0], 1), round(self.end[1], 1)],
                    alt=round(self.alt, 1), speed=round(self.speed, 1),
                    priority=self.priority,
                    cruise_t0=round(self.cruise_t0, 1),
                    cruise_t1=round(self.cruise_t1, 1))


# --------------------------------------------------------------------------- #
# 三个场景
# --------------------------------------------------------------------------- #
def headon(alt: float = 15.0) -> "Scenario":
    """两条平行走廊上各一对同高度对头机（同优先级 → 规则③各自右转）。"""
    plans = [
        FlightPlan("UAV-1", (-180.0, 0.0), (180.0, 0.0), alt=alt, speed=8.0,
                   label="东向干线"),
        FlightPlan("UAV-2", (180.0, 0.0), (-180.0, 0.0), alt=alt, speed=8.0,
                   label="西向干线"),
        FlightPlan("UAV-3", (-180.0, 60.0), (180.0, 60.0), alt=alt, speed=8.0,
                   label="东向支线"),
        FlightPlan("UAV-4", (180.0, 60.0), (-180.0, 60.0), alt=alt, speed=8.0,
                   label="西向支线"),
    ]
    return Scenario(
        "headon", "对头相遇",
        "两条平行走廊各有一对同高度对头机，中线相遇；"
        "同优先级按规则③各自右转 30° 互相从左側通过。",
        plans)


def cross(alt: float = 15.0) -> "Scenario":
    """四机同时穿过同一空域中心（同优先级 → 规则②左让右，旋转通过）。"""
    plans = [
        FlightPlan("UAV-1", (-180.0, 0.0), (180.0, 0.0), alt=alt, speed=8.0,
                   label="东行"),
        FlightPlan("UAV-2", (0.0, -180.0), (0.0, 180.0), alt=alt, speed=8.0,
                   label="北行"),
        FlightPlan("UAV-3", (180.0, 0.0), (-180.0, 0.0), alt=alt, speed=8.0,
                   label="西行"),
        FlightPlan("UAV-4", (0.0, 180.0), (0.0, -180.0), alt=alt, speed=8.0,
                   label="南行"),
    ]
    return Scenario(
        "cross", "十字交叉",
        "四架无人机从四个方向同时抵达空域中心，同优先级按规则②左让右，"
        "各自右转形成旋转通过。",
        plans)


def overtake(alt: float = 15.0) -> "Scenario":
    """两条平行走廊上各一对追越（规则④：追越机从左侧通过，前机保速保向）。"""
    plans = [
        FlightPlan("UAV-1", (-180.0, 0.0), (180.0, 0.0), alt=alt, speed=5.0,
                   label="慢速巡检（前机）"),
        FlightPlan("UAV-2", (-220.0, 0.0), (220.0, 0.0), alt=alt, speed=9.0,
                   label="快速配送（追越机）"),
        FlightPlan("UAV-3", (-180.0, 80.0), (180.0, 80.0), alt=alt, speed=5.0,
                   label="慢速巡检 B（前机）"),
        FlightPlan("UAV-4", (-220.0, 80.0), (220.0, 80.0), alt=alt, speed=9.0,
                   label="快速配送 B（追越机）"),
    ]
    return Scenario(
        "overtake", "同高度追越",
        "快机在正后方追上慢机（同航路同高度），规则④要求追越机从左侧"
        "偏置通过、前机保速保向；通过后按余弦剖面回到原航路。",
        plans)


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    plans: List[FlightPlan]


SCENARIOS: Dict[str, callable] = {
    "headon": headon,
    "cross": cross,
    "overtake": overtake,
}


def get_scenario(name: str, **kwargs) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("未知场景 %r，可选：%s" % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name](**kwargs)
