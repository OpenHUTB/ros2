#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三个可复现的城市低空噪声场景。

所有场景都是"城市低空点对点配送/巡检"：无人机从起降点以 8° 爬升角斜线
爬到巡航高度，平飞穿越城区后斜线下降。噪声冲突来自航迹从敏感点（学校/
医院/居民区）头顶或近旁掠过，而不是人为把飞机摆在敏感点上。

LAeq 归一到 1 小时观测期（``events_per_hour`` 为该走廊的小时航班密度，
住宅区上空走廊一天数百架是行业规划量级），单事件噪声 × 事件数 / 3600 s。

几何约束：爬升段水平投影 = alt / tan(8°) ≈ 4.26·alt，抬高 +60 m 后
投影 639 m，因此首末航段取 1600 m（容得下所有候选高度）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from .exposure import SensitivePoint
from .mitigate import MitigationParams
from .track import Flight

#: LAeq 观测期（s）：标准的 1 小时等效声级
OBS_PERIOD = 3600.0


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    flights: List[Flight]
    points: List[SensitivePoint]
    params: MitigationParams = field(default_factory=MitigationParams)
    events_per_hour: int = 1


def residential(alt: float = 30.0) -> Scenario:
    """两条航线沿居民区同走廊对飞，小区A几乎在航迹正下方。"""
    flights = [
        Flight("UAV-1", [(0, 0), (1600, 0)], alt=alt, t0=0.0, speed=10.0,
               climb_angle=8.0, label="东向配送（穿居民区）"),
        Flight("UAV-2", [(1600, 120), (0, 120)], alt=alt, t0=15.0, speed=10.0,
               climb_angle=8.0, label="西向配送（回程）"),
    ]
    points = [
        SensitivePoint("小区A", 800.0, 10.0, "residential", 1.5),
        SensitivePoint("小区B", 1200.0, -45.0, "residential", 1.0),
        SensitivePoint("小区C", 400.0, 65.0, "residential", 1.0),
    ]
    return Scenario(
        "residential", "居民区上空双机对飞",
        "两条平行航迹（间隔 120 m）沿居民区上空对飞，小时航班密度 60 架。"
        "小区A在两航迹中间（距各 10 m / 110 m），基线 30 m 巡航高度上"
        "LAeq 超过居住区昼间限值 60 dB。",
        flights, points, events_per_hour=60)


def school(alt: float = 30.0) -> Scenario:
    """航线从学校操场上空掠过，处在 200 m 缓冲区之内。"""
    flights = [
        Flight("UAV-1", [(-800, 0), (800, 0)], alt=alt, t0=0.0, speed=10.0,
               climb_angle=8.0, label="校园周边巡检"),
        Flight("UAV-2", [(-800, 420), (800, 420)], alt=alt, t0=20.0, speed=10.0,
               climb_angle=8.0, label="北侧干线配送"),
    ]
    points = [
        SensitivePoint("实验小学", 0.0, 12.0, "school", 2.5),
        SensitivePoint("小区D", 500.0, -40.0, "residential", 1.0),
    ]
    return Scenario(
        "school", "学校操场上空掠过",
        "巡检航迹从实验小学操场上空 12 m 侧向距离掠过（缓冲区 200 m 之内），"
        "小时航班密度 45 架；学校限值更严（55 dB，教学时段）且权重最高；"
        "另有一个居民点在航迹下方 40 m。",
        flights, points, events_per_hour=45)


def mixed(alt: float = 30.0) -> Scenario:
    """医院 + 学校 + 居民区混合敏感点，两机交叉穿越。"""
    flights = [
        Flight("UAV-1", [(-800, -60), (800, -60)], alt=alt, t0=0.0,
               speed=10.0, climb_angle=8.0, label="南线医疗配送"),
        Flight("UAV-2", [(800, 120), (-800, 120)], alt=alt, t0=8.0,
               speed=10.0, climb_angle=8.0, label="北线配送"),
    ]
    points = [
        SensitivePoint("中心医院", 0.0, 30.0, "hospital", 2.5),
        SensitivePoint("中学", -400.0, 150.0, "school", 2.0),
        SensitivePoint("小区E", 600.0, -20.0, "residential", 1.0),
    ]
    return Scenario(
        "mixed", "医院+学校+居民区混合",
        "两机交叉穿越（航迹 y=-60 / y=120，间隔 180 m），医院在两航迹中间"
        "（各距 90 m），学校在北线航迹旁 30 m，居民点在南线航迹旁 40 m——"
        "三类敏感点限值与权重各不相同，小时航班密度 45 架。",
        flights, points, events_per_hour=45)


SCENARIOS: Dict[str, callable] = {
    "residential": residential,
    "school": school,
    "mixed": mixed,
}


def get_scenario(name: str, **kwargs) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("未知场景 %r，可选：%s" % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name](**kwargs)
