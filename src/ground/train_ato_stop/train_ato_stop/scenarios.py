"""四个场景：平直道、中途限速、连续坡道，以及一个晚制动的对照。

对照组的意义和别处一样：只给出"停准了"的数字，没办法说明是曲线起了作用
还是线路本来就宽松。晚制动那一组把制动点推迟到距离不足的位置，ATP 会介入，
但仍然停不住——这样才看得出规划曲线到底在防什么。
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .train import Segment


@dataclass
class Scenario:
    name: str
    title: str
    title_en: str
    segments: list[Segment]
    stop_x: float
    note: str = ''
    # 设了就改成"人工驾驶"策略：一直按限速跑，到距停车点这个距离才动手制动
    manual_brake_distance: float | None = None

    @property
    def station_name(self) -> str:
        return 'B'

    def limit_at(self, x: float) -> float:
        from .train import speed_limit_at
        return speed_limit_at(self.segments, x)

    def max_limit(self) -> float:
        return max(seg.limit_kph for seg in self.segments)


def flat() -> Scenario:
    """平直道，2.4 km 一站直达。"""
    return Scenario(
        name='flat',
        title='平直道一站直达',
        title_en='FLAT TRACK',
        segments=[Segment(0.0, 2400.0, 160.0, grade=0.0)],
        stop_x=2400.0,
        note='全线 160 km/h，无坡道，检验基本的加速—巡航—制动链',
    )


def restriction() -> Scenario:
    """中途一段 80 km/h 限速，考验提前减速与再加速。"""
    return Scenario(
        name='restriction',
        title='中途临时限速',
        title_en='SPEED RESTRICTION',
        segments=[
            Segment(0.0, 800.0, 160.0, grade=0.0),
            Segment(800.0, 1300.0, 80.0, grade=0.0),
            Segment(1300.0, 2800.0, 160.0, grade=0.0),
        ],
        stop_x=2800.0,
        note='800–1300 m 限速 80 km/h，进限速区前必须提前减速，出区后再加速',
    )


def grade() -> Scenario:
    """先上坡后下坡，下坡段制动距离会明显变长。"""
    return Scenario(
        name='grade',
        title='连续坡道',
        title_en='GRADIENT',
        segments=[
            Segment(0.0, 1000.0, 160.0, grade=0.0),
            Segment(1000.0, 1700.0, 160.0, grade=8.0),
            Segment(1700.0, 3000.0, 160.0, grade=-12.0),
        ],
        stop_x=3000.0,
        note='1700 m 起 12‰ 下坡，坡道分力抵消一部分制动力，制动距离变长',
    )


def late_brake() -> Scenario:
    """对照：不做曲线规划，一直跑到距站 700 m 才制动。"""
    return Scenario(
        name='late_brake',
        title='晚制动（对照）',
        title_en='LATE BRAKING',
        segments=[Segment(0.0, 2400.0, 160.0, grade=0.0)],
        stop_x=2400.0,
        note='制动点推迟到距站 520 m，从 160 km/h 刹停实际需要约 660 m，必然越位',
        manual_brake_distance=520.0,
    )


def all_scenarios() -> list[Scenario]:
    return [flat(), restriction(), grade(), late_brake()]
