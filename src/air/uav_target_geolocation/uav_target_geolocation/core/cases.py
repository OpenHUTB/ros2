#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算例定义：同一套算法换三种观察站几何，看误差怎么变。

目标都是地面上的同一点，无人机在 78~100 m 斜距上悬停观察，差别只在
"站在哪、几个站、站与站张开多大角度"。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from .geometry import pairwise_min_angle_deg, station_position

#: 目标点（世界系，地面）
TARGET = (0.0, 0.0, 0.0)
#: 观察高度（m）
HEIGHT_M = 80.0
#: 观察站到目标的地面水平距离（m）
RADIUS_M = 60.0
#: 万向节视轴逐帧抖动（1σ，度）——云台稳定环的残余抖动量级
SIGMA_DEG = 0.30
#: 单站视轴零位偏置（1σ，度）——标定残差，整架次不变，平均不掉
BIAS_DEG = 0.20
#: 悬停时的水平外力（N）。定点保持要抵消它，机体必然倾转几度——
#: "拿指令姿态（水平）当真实姿态"的误差就从这里来
WIND_N = (1.0, 0.0)


@dataclass(frozen=True)
class Station:
    """一个观察站：方位角 + 水平距离 + 高度。"""

    azimuth_deg: float
    radius_m: float = RADIUS_M
    height_m: float = HEIGHT_M

    @property
    def position(self) -> np.ndarray:
        return station_position(self.azimuth_deg, self.radius_m, self.height_m)


@dataclass(frozen=True)
class GeoCase:
    """一个算例：目标 + 观察站布局 + 观测参数。"""

    name: str
    title: str
    stations: Tuple[Station, ...]
    target: Tuple[float, float, float] = TARGET
    sigma_deg: float = SIGMA_DEG
    bias_deg: float = BIAS_DEG
    wind_n: Tuple[float, float] = WIND_N   # 悬停时的水平外力（N），迫使机体倾转
    hover_s: float = 3.0            # 每站稳定悬停时长（采样窗）
    samples: int = 120              # 每站采样帧数
    legacy_level_deg: float = 0.0   # "假定机体水平"的对照算例用

    def __post_init__(self) -> None:
        if not self.stations:
            raise ValueError(f"{self.name}: 至少要有一个观察站")
        if self.sigma_deg < 0.0:
            raise ValueError(f"{self.name}: sigma_deg 不能为负")
        if self.bias_deg < 0.0:
            raise ValueError(f"{self.name}: bias_deg 不能为负")
        if self.samples < 1:
            raise ValueError(f"{self.name}: samples 至少为 1")

    @property
    def azimuths(self) -> Tuple[float, ...]:
        return tuple(s.azimuth_deg for s in self.stations)

    @property
    def intersection_deg(self) -> float:
        """站与站之间最小的水平开角（度）。单站为 nan。"""
        return pairwise_min_angle_deg(self.azimuths)

    @property
    def n_stations(self) -> int:
        return len(self.stations)


CASES: Tuple[GeoCase, ...] = (
    GeoCase(
        name="single_station",
        title="单站定位（60 m 外、80 m 高）",
        stations=(Station(azimuth_deg=0.0),),
    ),
    GeoCase(
        name="two_station_orthogonal",
        title="双站交会（开角 90°）",
        stations=(Station(azimuth_deg=0.0), Station(azimuth_deg=90.0)),
    ),
    GeoCase(
        name="two_station_shallow",
        title="双站浅交会（开角 12°）",
        stations=(Station(azimuth_deg=0.0), Station(azimuth_deg=12.0)),
    ),
)


def get_case(name: str) -> GeoCase:
    for c in CASES:
        if c.name == name:
            return c
    raise KeyError(f"未知算例 {name!r}，可选：{[c.name for c in CASES]}")


def sweep_case(angle_deg: float, sigma_deg: float = SIGMA_DEG,
               bias_deg: float = BIAS_DEG) -> GeoCase:
    """生成"双站开角 = angle_deg"的算例，用于画误差随几何变化的曲线。"""
    if angle_deg <= 0.0 or angle_deg > 180.0:
        raise ValueError(f"开角必须在 (0, 180]，收到 {angle_deg}")
    return GeoCase(
        name=f"two_station_{angle_deg:g}deg",
        title=f"双站交会（开角 {angle_deg:g}°）",
        stations=(Station(0.0), Station(angle_deg)),
        sigma_deg=sigma_deg,
        bias_deg=bias_deg,
    )
