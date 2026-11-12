"""四个考察半主动悬架的工况。

前三个是单事件工况，分别对应"过减速带不颠"、"急刹不点头"、"过弯不侧倾"；
第四个是 ISO 8608 C 级随机路面上的持续激励，看长时间统计意义下的舒适性与接地性。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .params import GRAVITY
from .road import BumpRoad, FlatRoad, IsoRoad, Road


def _ramp(t: float, start: float, rise: float, level: float) -> float:
    """在 start 之后用 rise 秒线性升到 level，之后保持。"""
    if t <= start:
        return 0.0
    if t < start + rise:
        return level * (t - start) / rise
    return level


@dataclass(frozen=True)
class Scenario:
    name: str
    title: str          # 图上用的英文标题（点阵字只支持大写字母）
    subtitle: str
    road: Road
    speed: float        # m/s
    duration: float
    accel_x: float = 0.0
    accel_y: float = 0.0
    ramp_start: float = 0.0
    ramp_rise: float = 0.3

    def velocity(self, t: float) -> float:
        """车速，m/s。制动工况里随减速度衰减，不出现倒车。"""
        if self.accel_x >= 0.0:
            return self.speed
        return max(0.0, self.speed + self.accel_x * max(0.0, t - self.ramp_start))

    def travel(self, t: float) -> float:
        """从起点走过的距离，m。"""
        if self.accel_x >= 0.0:
            return self.speed * t
        dt = max(0.0, t - self.ramp_start)
        early = min(t, self.ramp_start) * self.speed
        return early + self.speed * dt + 0.5 * self.accel_x * dt * dt

    def longitudinal(self, t: float) -> float:
        return _ramp(t, self.ramp_start, self.ramp_rise, self.accel_x)

    def lateral(self, t: float) -> float:
        return _ramp(t, self.ramp_start, self.ramp_rise, self.accel_y)


def speed_bump() -> Scenario:
    return Scenario(
        name="speed_bump",
        title="SPEED BUMP",
        subtitle="20 KM/H, 50 MM BUMP",
        road=BumpRoad(height=0.05, length=0.4, start=3.0),
        speed=20.0 / 3.6,
        duration=2.2,
    )


def brake_dive() -> Scenario:
    return Scenario(
        name="brake_dive",
        title="BRAKE DIVE",
        subtitle="100 KM/H, 0.6 G DECEL",
        road=FlatRoad(),
        speed=100.0 / 3.6,
        duration=2.4,
        accel_x=-0.6 * GRAVITY,
        ramp_start=0.3,
        ramp_rise=0.35,
    )


def cornering_roll() -> Scenario:
    return Scenario(
        name="cornering_roll",
        title="CORNERING ROLL",
        subtitle="80 KM/H, 0.4 G LATERAL",
        road=FlatRoad(),
        speed=80.0 / 3.6,
        duration=2.6,
        accel_y=0.4 * GRAVITY,
        ramp_start=0.3,
        ramp_rise=0.45,
    )


def iso_rough() -> Scenario:
    return Scenario(
        name="iso_rough",
        title="ISO 8608 CLASS C",
        subtitle="80 KM/H RANDOM ROAD",
        road=IsoRoad(grade="C"),
        speed=80.0 / 3.6,
        duration=6.0,
    )


def all_scenarios() -> list:
    return [speed_bump(), brake_dive(), cornering_roll(), iso_rough()]


def by_name(name: str) -> Scenario:
    for scenario in all_scenarios():
        if scenario.name == name:
            return scenario
    raise KeyError("未知工况: %s" % name)


def road_inputs(scenario: Scenario, spec, t: float) -> tuple:
    """返回 (road, road_rate)，四个轮子的路面高度与变化率。"""
    x_front = scenario.travel(t)
    x_rear = x_front - spec.wheelbase
    heights = []
    rates = []
    for corner in range(4):
        x = x_front if corner in (0, 1) else x_rear
        heights.append(scenario.road.height(x))
        # 路面变化率 = dz/dx * dx/dt
        rates.append(scenario.road.slope(x) * scenario.velocity(t))
    return tuple(heights), tuple(rates)


def phase_delay(scenario: Scenario, spec) -> float:
    """后轮相对前轮的时间差，s。"""
    return spec.wheelbase / max(0.5, scenario.speed)


def is_step_case(scenario: Scenario) -> bool:
    return scenario.name in ("brake_dive", "cornering_roll")


def quiescent_seconds(scenario: Scenario) -> float:
    """平路工况先跑一小段让悬架静止。"""
    return 0.0 if is_step_case(scenario) else 0.2


def degrees(rad: float) -> float:
    return rad * 180.0 / math.pi
