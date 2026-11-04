"""四个会遇场景：对遇、交叉、追越、三船同时会遇。

坐标系：x 向东、y 向北，单位 m；航向 0° 为正北、顺时针为正。
每条船的终点都设在起始航线上，避让结束后会自己拐回来。
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .colreg import Vessel


@dataclass
class Scenario:
    name: str
    title: str
    vessels: list[Vessel]
    goals: list[tuple[float, float]]
    title_en: str = ''
    t_max: float = 600.0
    note: str = ''

    def __post_init__(self) -> None:
        if len(self.vessels) != len(self.goals):
            raise ValueError(f'{self.name}: 船数与终点数不一致')


def head_on() -> Scenario:
    """Rule 14：两船正对开，各自向右转。"""
    return Scenario(
        name='head_on',
        title='对遇 Head-on (Rule 14)',
        vessels=[
            Vessel('USV-A', x=0.0, y=0.0, heading=0.0, speed=9.0),
            Vessel('MV-B', x=0.0, y=3000.0, heading=180.0, speed=9.0),
        ],
        goals=[(0.0, 6000.0), (0.0, -3000.0)],
        title_en='HEAD-ON (RULE 14)',
        t_max=400.0,
        note='两船都以 9 m/s 相向而行，间距 3 km，理论上 DCPA = 0',
    )


def crossing() -> Scenario:
    """Rule 15：目标从右舷横穿，本船是让路船。"""
    return Scenario(
        name='crossing',
        title='交叉相遇 Crossing (Rule 15)',
        vessels=[
            Vessel('USV-A', x=0.0, y=0.0, heading=0.0, speed=8.0),
            Vessel('MV-B', x=1200.0, y=1200.0, heading=270.0, speed=8.0),
        ],
        goals=[(0.0, 6000.0), (-3000.0, 1200.0)],
        title_en='CROSSING (RULE 15)',
        t_max=400.0,
        note='B 船从 A 船右舷约 68° 方位横穿，两船同时到达航迹交点',
    )


def overtaking() -> Scenario:
    """Rule 13：本船从后方追上慢船，本船是让路船。"""
    return Scenario(
        name='overtaking',
        title='追越 Overtaking (Rule 13)',
        vessels=[
            Vessel('USV-A', x=0.0, y=0.0, heading=0.0, speed=12.0),
            Vessel('MV-B', x=0.0, y=1500.0, heading=0.0, speed=5.0),
        ],
        goals=[(0.0, 6000.0), (0.0, 6000.0)],
        title_en='OVERTAKING (RULE 13)',
        t_max=420.0,
        note='A 船比 B 船快 7 m/s，从正后方追越，被追越船保持航向航速',
    )


def multi() -> Scenario:
    """三船：A 右舷有交叉来船（本船让路），左舷另有交叉来船（对方让路）。"""
    return Scenario(
        name='multi',
        title='三船同时会遇',
        vessels=[
            Vessel('USV-A', x=0.0, y=0.0, heading=0.0, speed=8.0),
            Vessel('MV-B', x=1000.0, y=1000.0, heading=270.0, speed=8.0),
            Vessel('USV-C', x=-1600.0, y=1600.0, heading=90.0, speed=8.0),
        ],
        goals=[(0.0, 6000.0), (-3000.0, 1000.0), (3000.0, 1600.0)],
        title_en='MULTI-SHIP ENCOUNTER',
        t_max=400.0,
        note='B 在 A 右舷、C 在 A 左舷，A 只需为 B 让路，C 自行避让 A',
    )


ALL = [head_on, crossing, overtaking, multi]


def by_name(name: str) -> Scenario:
    for factory in ALL:
        scenario = factory()
        if scenario.name == name:
            return scenario
    raise KeyError(f'未知场景: {name}')
