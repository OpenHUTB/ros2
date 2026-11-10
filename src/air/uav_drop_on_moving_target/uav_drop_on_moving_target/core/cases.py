#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三个内置算例：直行车队、侧风、中途停车。

一行摘要：算例把"车辆轨迹 + 无人机航迹 + 投放条件"打成一个对象，CLI、
测试与闭环共用同一份定义，避免几处各写一套数字。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .release import DropCfg
from .track import PolylineTrack
from .vehicle import Vehicle


@dataclass(frozen=True)
class ConvoyCase:
    """一个完整算例。"""

    name: str
    title: str
    note: str
    vehicle: Vehicle
    track: PolylineTrack
    cfg: DropCfg


CONVOY_STRAIGHT = ConvoyCase(
    name="convoy_straight",
    title="直行车队（无风）",
    note="车队 6 m/s 沿直路前进，无人机 10 m/s 从后方追上。货要落 2.47 s，"
         "这段时间车队又走了 14.8 m——瞄准它此刻的位置，落点必然偏出接受半径。",
    vehicle=Vehicle("convoy-A", ((0.0, 0.0), (700.0, 0.0)), 6.0),
    track=PolylineTrack(((-80.0, 0.0), (560.0, 0.0)), 10.0),
    cfg=DropCfg(alt_m=30.0, speed=10.0, accept_radius_m=6.0, wind=(0.0, 0.0)),
)

CONVOY_CROSSWIND = ConvoyCase(
    name="convoy_crosswind",
    title="6 m/s 侧风下的车队",
    note="侧风 6 m/s 让货在空中横飘 11.9 m，比接受半径大一倍；车队自己还在走"
         "14.8 m。两种偏差必须分开补偿：风偏把瞄准点反向平移，车队位移用未来"
         "位置预测。",
    vehicle=Vehicle("convoy-B", ((0.0, 20.0), (700.0, 20.0)), 6.0),
    track=PolylineTrack(((-80.0, 20.0), (560.0, 20.0)), 10.0),
    cfg=DropCfg(alt_m=30.0, speed=10.0, accept_radius_m=6.0, wind=(6.0, 0.0)),
)

CONVOY_STOPGO = ConvoyCase(
    name="convoy_stopgo",
    title="中途停车 6 s 再起步",
    note="车队在 (260, 0) 停车 6 s 卸货再走。按总里程/车速算到达时刻会把这 6 s"
         "当没发生，释放时刻正好落在停车窗口之后，落点偏出 36 m；按真实轨迹"
         "（含停留）解算则选在车队停稳的窗口投放。",
    vehicle=Vehicle("convoy-C", ((0.0, 0.0), (260.0, 0.0), (560.0, 0.0)), 6.0,
                    dwell_s=((1, 6.0),)),
    track=PolylineTrack(((-80.0, 0.0), (560.0, 0.0)), 10.0),
    cfg=DropCfg(alt_m=30.0, speed=10.0, accept_radius_m=6.0, wind=(0.0, 0.0)),
)

CASES: Tuple[ConvoyCase, ...] = (CONVOY_STRAIGHT, CONVOY_CROSSWIND, CONVOY_STOPGO)


def make_case(name: str) -> ConvoyCase:
    for c in CASES:
        if c.name == name:
            return c
    raise KeyError("unknown case %r, available: %s"
                   % (name, ", ".join(c.name for c in CASES)))
