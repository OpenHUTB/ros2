#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三个可复现的低空运行场景。

所有场景都是"城市低空点对点飞行"：无人机从起降点垂直爬升到巡航高度，
平飞穿越空域后垂直下降。冲突来自航路交叉、对头与会遇，而不是人为把
飞机摆在一起。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Sequence

from typing import Optional

from .conflict import Separation
from .plan import Plan
from .resolve import Limits


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    plans: List[Plan]
    sep: Separation = field(default_factory=Separation)
    limits: Optional[Limits] = None


def crossing(alt: float = 60.0) -> Scenario:
    """四条航路在同一时刻穿过同一空域中心，外加一架同向跟随机。"""
    plans = [
        Plan("UAV-1", [(-220, 0), (220, 0)], alt=alt, t0=0.0, speed=8.0,
             label="东西向配送"),
        Plan("UAV-2", [(0, -220), (0, 220)], alt=alt, t0=0.0, speed=8.0,
             label="南北向配送"),
        Plan("UAV-3", [(220, 0), (-220, 0)], alt=alt, t0=0.0, speed=8.0,
             label="西向回程"),
        Plan("UAV-4", [(0, 220), (0, -220)], alt=alt, t0=0.0, speed=8.0,
             label="南向回程"),
        Plan("UAV-5", [(-180, 180), (180, -180)], alt=alt, t0=10.0, speed=8.0,
             label="斜向巡检"),
    ]
    return Scenario(
        "crossing", "十字交叉 + 斜向穿越",
        "四条航路在同一时刻穿越中心点，第五架沿对角线斜穿同一空域。"
        "中心点四机同时到达，属于最坏情况的空域拥塞。",
        plans)


def corridor(alt: float = 60.0) -> Scenario:
    """同一条走廊里对头 + 追越，含一架医疗急救机（不可延误）。

    四架飞机沿同一条东西向走廊飞行，但起降点分布在不同的停机位，
    因此冲突全部来自空中（对头、追越），而不是两架停在同一个起降点。
    """
    plans = [
        Plan("UAV-1", [(-300, 0), (300, 0)], alt=alt, t0=0.0, speed=8.0,
             label="东向干线"),
        Plan("UAV-2", [(300, 0), (-300, 0)], alt=alt, t0=20.0, speed=8.0,
             label="西向干线"),
        Plan("UAV-3", [(-150, 0), (150, 0)], alt=alt, t0=40.0, speed=5.0,
             label="慢速巡检"),
        Plan("UAV-4", [(-60, 0), (240, 0)], alt=alt, t0=55.0, speed=12.0,
             priority=10, label="医疗急救（不可延误）"),
    ]
    return Scenario(
        "corridor", "走廊对头 + 追越",
        "同一条东西向走廊内两机对头飞行、快机追越慢机；"
        "另有一架高优先级医疗急救机，解脱时不允许被延误。",
        plans)


def metro(alt: float = 60.0) -> Scenario:
    """网格状城市低空：六架飞机沿两横两纵航路飞行。"""
    plans = [
        Plan("UAV-1", [(-260, -60), (260, -60)], alt=alt, t0=0.0, speed=9.0,
             label="南横线东行"),
        Plan("UAV-2", [(260, 60), (-260, 60)], alt=alt, t0=2.0, speed=9.0,
             label="北横线西行"),
        Plan("UAV-3", [(-120, -260), (-120, 260)], alt=alt, t0=0.0, speed=8.0,
             label="西纵线北行"),
        Plan("UAV-4", [(120, 260), (120, -260)], alt=alt, t0=3.0, speed=8.0,
             label="东纵线南行"),
        Plan("UAV-5", [(-260, 0), (260, 0)], alt=alt, t0=8.0, speed=7.0,
             label="中横线东行"),
        Plan("UAV-6", [(0, 260), (0, -260)], alt=alt, t0=10.0, speed=7.0,
             label="中纵线南行"),
    ]
    return Scenario(
        "metro", "网格状多机空域",
        "六架飞机沿两横两纵航路飞行，交叉点多、时间错开但仍有重叠，"
        "用于检验多冲突同时存在时的消解能力。",
        plans)


def slot(alt: float = 60.0) -> Scenario:
    """高峰时段：起飞时隙已排定，不允许地面等待，只能改高度或绕行。"""
    plans = [
        Plan("UAV-1", [(-200, -20), (200, -20)], alt=alt, t0=0.0, speed=8.0,
             label="东行干线"),
        Plan("UAV-2", [(20, -200), (20, 200)], alt=alt, t0=0.0, speed=8.0,
             label="北行干线"),
        Plan("UAV-3", [(200, 20), (-200, 20)], alt=alt, t0=0.0, speed=8.0,
             label="西行干线"),
    ]
    return Scenario(
        "slot", "固定时隙（禁止地面等待）",
        "三架飞机在高峰时段已排定起飞时隙，不允许地面等待，"
        "解脱只能通过高度层或侧向绕行完成。",
        plans,
        limits=Limits(max_delay=0.0))


SCENARIOS: Dict[str, callable] = {
    "crossing": crossing,
    "corridor": corridor,
    "metro": metro,
    "slot": slot,
}


def get_scenario(name: str, **kwargs) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("未知场景 %r，可选：%s" % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name](**kwargs)
