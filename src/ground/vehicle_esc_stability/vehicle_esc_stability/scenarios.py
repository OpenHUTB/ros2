"""四个用于验证 ESC 的工况。

双移线和正弦停滞考察转向输入的响应；对开路面制动与湿滑双移线是
用来区分「有 ESC / 无 ESC」的极限工况：前者靠左右附着差把车拽偏，
后者是低附着下后轴先饱和、车尾甩出去。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from .driver import ReferencePath
from .dynamics import RoadSpec

DEG = math.pi / 180.0


@dataclass
class Scenario:
    """一个工况的输入定义。"""

    name: str
    title: str
    title_en: str
    road: RoadSpec
    initial_speed: float
    duration: float
    steer_fn: Callable[[float], float]
    brake_fn: Callable[[float], float] = lambda t: 0.0
    hold_speed: bool = True
    notes: str = ""
    path: ReferencePath | None = None
    """给定参考路径时，方向盘由纯跟踪驾驶员给出，steer_fn 不再使用。"""

    def steer(self, t: float) -> float:
        return self.steer_fn(t)

    def brake(self, t: float) -> float:
        return self.brake_fn(t)

    @property
    def title_for_render(self) -> str:
        """点阵字只有 ASCII 字形，图里用英文标题。"""
        return self.title_en


def _lobe(t: float, start: float, width: float, amplitude: float) -> float:
    """半正弦波，用于构造双移线的两段转向。"""
    local = t - start
    if local < 0.0 or local > width:
        return 0.0
    return amplitude * math.sin(math.pi * local / width)


def lane_change() -> Scenario:
    """双移线：驾驶员按参考路径（横向平移 3.5 m）跟踪，40 m 内完成换道。"""

    path = ReferencePath.double_lane_change(offset=3.5, x_start=10.0, length=40.0)

    return Scenario(
        name="lane_change",
        title="双移线换道（闭环驾驶员）",
        title_en="DOUBLE LANE CHANGE",
        road=RoadSpec(0.90, 0.90),
        initial_speed=26.8,
        duration=8.0,
        steer_fn=lambda t: 0.0,
        path=path,
        notes="96 km/h、μ=0.9，40 m 内横向平移 3.5 m；方向盘由纯跟踪驾驶员给出",
    )


def sine_dwell() -> Scenario:
    """正弦停滞：0.5 Hz 正弦转向 4 s，随后方向盘回正保持。"""

    amp = 2.2 * DEG
    freq = 0.5

    def steer(t: float) -> float:
        if t > 4.0:
            return 0.0
        return amp * math.sin(2.0 * math.pi * freq * t)

    return Scenario(
        name="sine_dwell",
        title="正弦停滞（ISO 7401 风格）",
        title_en="SINE WITH DWELL",
        road=RoadSpec(0.90, 0.90),
        initial_speed=27.8,
        duration=7.0,
        steer_fn=steer,
        notes="100 km/h、0.5 Hz、峰值 2.2°，用于看横摆角速度的跟踪误差与残余摆动",
    )


def mu_split() -> Scenario:
    """对开路面制动：左侧高附着、右侧低附着，全力制动。"""

    def brake(t: float) -> float:
        if t < 0.5:
            return 0.0
        return 0.85

    return Scenario(
        name="mu_split",
        title="对开路面全力制动（μ-split）",
        title_en="SPLIT-MU BRAKING",
        road=RoadSpec(0.95, 0.30),
        initial_speed=27.8,
        duration=12.0,
        steer_fn=lambda t: 0.0,
        brake_fn=brake,
        hold_speed=False,
        notes="100 km/h 起全力制动，左轮 μ=0.95、右轮 μ=0.30",
    )


def wet_lane_change() -> Scenario:
    """湿滑路面（μ=0.5）双移线：附着只有干燥路面的一半，极限提前到来。"""

    path = ReferencePath.double_lane_change(offset=3.5, x_start=10.0, length=40.0)

    return Scenario(
        name="wet_lane_change",
        title="湿滑路面双移线",
        title_en="WET DOUBLE LANE CHANGE",
        road=RoadSpec(0.50, 0.50),
        initial_speed=18.0,
        duration=9.0,
        steer_fn=lambda t: 0.0,
        path=path,
        notes="65 km/h、μ=0.5，同样是 40 m 内平移 3.5 m",
    )


def all_scenarios() -> list[Scenario]:
    return [lane_change(), sine_dwell(), mu_split(), wet_lane_change()]


def by_name(name: str) -> Scenario:
    for scenario in all_scenarios():
        if scenario.name == name:
            return scenario
    raise KeyError(f"未知工况：{name}")
