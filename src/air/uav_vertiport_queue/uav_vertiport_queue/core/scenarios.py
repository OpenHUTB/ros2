#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个可复现的起降场运行场景与场地布局。

场地布局（ENU，单位 m）::

    起飞位 T1(0, 0)   T2(12, 0)          降落位 L1(0, 80)  L2(12, 80)
    停机坪 P1..P4：(40..85, 25)          离场航路点 (300, 0)
    进场定位点 (-300, 80)

场景语义：每架 UAV 一次离场 + 一次进场（``架次`` = 请求条数）。
拥挤来自请求时刻扎堆，而不是人为构造。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence

from .queue import ARRIVAL, DEPARTURE, QueueConfig, Request

# --------------------------------------------------------------------------- #
# 场地布局（坐标 m）
# --------------------------------------------------------------------------- #
LAYOUT = dict(
    pads_departure={"T1": (0.0, 0.0), "T2": (12.0, 0.0)},
    pads_arrival={"L1": (0.0, 80.0), "L2": (12.0, 80.0)},
    apron={"P1": (40.0, 25.0), "P2": (55.0, 25.0),
           "P3": (70.0, 25.0), "P4": (85.0, 25.0)},
    departure_exit=(300.0, 0.0),
    arrival_fix=(-300.0, 80.0),
)


def departure_exit_xy() -> Sequence[float]:
    return LAYOUT["departure_exit"]


def arrival_fix_xy() -> Sequence[float]:
    return LAYOUT["arrival_fix"]


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    requests: List[Request]
    strategy: str = "interval"          # 排队策略（medical 用 priority）
    config: QueueConfig = field(default_factory=QueueConfig)


def _dep(uid: str, t: float, priority: int = 0, label: str = "") -> Request:
    return Request(uid=uid, kind=DEPARTURE, t_request=t,
                   priority=priority, label=label)


def _arr(uid: str, t: float, priority: int = 0, label: str = "") -> Request:
    return Request(uid=uid, kind=ARRIVAL, t_request=t,
                   priority=priority, label=label)


# --------------------------------------------------------------------------- #
# 场景定义
# --------------------------------------------------------------------------- #
def light() -> Scenario:
    """8 架次（8 架 UAV 各 1 个架次）：离场时刻扎堆导致排队。"""
    requests = [
        _dep("UAV-1", 0.0, label="东向配送"),
        _dep("UAV-2", 20.0, label="北向配送"),
        _dep("UAV-3", 70.0, label="巡检"),
        _dep("UAV-4", 110.0, label="巡检"),
        _arr("UAV-5", 160.0, label="返场配送"),
        _arr("UAV-6", 190.0, label="返场配送"),
        _arr("UAV-7", 220.0, label="返场巡检"),
        _arr("UAV-8", 250.0, label="返场巡检"),
    ]
    return Scenario(
        "light", "轻度流量（8 架次）",
        "8 架 UAV 各执行 1 个架次：前 4 架离场时刻间隔仅 20~40 s，"
        "小于离场最小间隔 60 s，必须排队；进场同理受 90 s 间隔约束。",
        requests)


def rush() -> Scenario:
    """16 架次（8 架 UAV 各离场 + 进场）：高峰通勤流。"""
    requests: List[Request] = []
    for i in range(8):
        uid = "UAV-%d" % (i + 1)
        requests.append(_dep(uid, 35.0 * i,
                             label="早高峰出港 %d" % (i + 1)))
    for i in range(8):
        uid = "UAV-%d" % (i + 1)
        requests.append(_arr(uid, 300.0 + 35.0 * i,
                             label="早高峰进港 %d" % (i + 1)))
    return Scenario(
        "rush", "早高峰（16 架次）",
        "8 架 UAV 先后出港再返场，离场请求间隔 35 s、进场请求间隔 35 s，"
        "均远小于 60/90 s 最小间隔，排队延误显著。",
        requests)


def medical() -> Scenario:
    """12 架次（6 架 UAV），含一架医疗急救 UAV（priority=10）插队。"""
    requests = [
        _dep("UAV-1", 0.0, label="普通配送"),
        _dep("UAV-2", 15.0, label="普通配送"),
        _dep("UAV-3", 30.0, priority=10, label="医疗急救（送血）"),
        _dep("UAV-4", 45.0, label="普通配送"),
        _arr("UAV-1", 150.0, label="普通返场"),
        _arr("UAV-2", 170.0, label="普通返场"),
        _arr("UAV-3", 210.0, priority=10, label="急救返场"),
        _arr("UAV-4", 230.0, label="普通返场"),
        _dep("UAV-5", 300.0, label="普通配送"),
        _dep("UAV-6", 315.0, label="普通配送"),
        _arr("UAV-5", 420.0, label="普通返场"),
        _arr("UAV-6", 440.0, label="普通返场"),
    ]
    return Scenario(
        "medical", "急救插队（12 架次）",
        "6 架 UAV 共 12 个架次：UAV-3 为医疗急救（priority=10），"
        "在 0~45 s 的离场机流中要求优先放行——priority 策略下急救机"
        "不被延误，普通机流为其让行。",
        requests, strategy="priority")


def saturation() -> Scenario:
    """20 架次（10 架 UAV）：请求率超容量，饱和压测。"""
    requests: List[Request] = []
    for i in range(10):
        uid = "UAV-%d" % (i + 1)
        requests.append(_dep(uid, 20.0 * i, label="饱和压测出港 %d" % (i + 1)))
    for i in range(10):
        uid = "UAV-%d" % (i + 1)
        requests.append(_arr(uid, 200.0 + 20.0 * i,
                             label="饱和压测进港 %d" % (i + 1)))
    return Scenario(
        "saturation", "饱和压测（20 架次）",
        "10 架 UAV 出港/进港请求各间隔 20 s，请求率远超 60/90 s 间隔"
        "决定的容量上限，队列持续增长，用于观测最大排队长度与饱和行为。",
        requests)


SCENARIOS: Dict[str, Callable[[], Scenario]] = {
    "light": light,
    "rush": rush,
    "medical": medical,
    "saturation": saturation,
}


def get_scenario(name: str) -> Scenario:
    if name not in SCENARIOS:
        raise KeyError("未知场景 %r，可选：%s" % (name, ", ".join(SCENARIOS)))
    return SCENARIOS[name]()
