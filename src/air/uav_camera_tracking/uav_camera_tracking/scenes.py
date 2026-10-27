# -*- coding: utf-8 -*-
"""scenes.py —— 四个测试场景。

同一个跟踪器要在"静止 / 匀速圆周 / 遮挡重现 / 阵风扰动"四种情形下都保持
目标在视野内，才能说明闭环不是只对一种工况调出来的。
"""
from dataclasses import dataclass, field
from typing import Callable, Optional, Tuple

import numpy as np

from . import params as P


@dataclass
class Scene:
    key: str
    name: str
    desc: str
    duration: float
    uav0: Tuple[float, float, float]
    yaw0: float
    target_fn: Callable[[float], Tuple[float, float]]
    walls: Tuple[Tuple[float, float, float, float, float], ...] = field(default_factory=tuple)
    distractors: Tuple[Tuple[float, float], ...] = field(default_factory=tuple)
    wind_fn: Optional[Callable[[float], Tuple[float, float, float]]] = None

    def steps(self):
        return int(round(self.duration / P.DT))

    def wind(self, t):
        if self.wind_fn is None:
            return np.zeros(3)
        return np.asarray(self.wind_fn(t), dtype=float)


def _circle(cx, cy, r, omega, phase=0.0):
    def fn(t):
        a = phase + omega * t
        return (cx + r * np.cos(a), cy + r * np.sin(a))
    return fn


def _line_then_back(p0, p1, period):
    def fn(t):
        s = (t % period) / period
        tri = 2 * s if s < 0.5 else 2 * (1 - s)   # 0->1->0 三角波
        return (p0[0] + (p1[0] - p0[0]) * tri, p0[1] + (p1[1] - p0[1]) * tri)
    return fn


def _gust(t):
    """阵风：两个不同频率分量叠加，模拟突风与持续侧风（幅值约 0.5 m/s²）。"""
    return (0.52 * np.sin(0.9 * t) + 0.24 * np.sin(2.7 * t + 1.1),
            0.44 * np.sin(1.3 * t + 0.4),
            0.18 * np.sin(1.7 * t))


def build_scenes():
    """返回 4 个场景（顺序即文档与结果表中的顺序）。"""
    return [
        Scene(
            key='static', name='静止目标',
            desc='目标静止，无人机从 10 m 外接近并稳定保持跟飞距离',
            duration=18.0, uav0=(0.0, 0.0, P.H_DES), yaw0=0.0,
            target_fn=lambda t: (10.0, 0.0),
        ),
        Scene(
            key='moving', name='匀速圆周运动目标',
            desc='目标沿半径 5 m 圆周以 0.22 rad/s 匀速运动，需持续转向跟随',
            duration=22.0, uav0=(6.0, 0.0, P.H_DES), yaw0=0.0,
            target_fn=_circle(11.0, 0.0, 5.0, 0.22),
            distractors=((11.0, 6.5),),
        ),
        Scene(
            key='occlusion', name='立柱遮挡后重捕获',
            desc='目标横穿一排立柱，被遮挡 3 次（每次约 0.6 s），'
                 '每次重现都要重新检测并恢复跟踪',
            duration=30.0, uav0=(2.0, -6.0, P.H_DES), yaw0=0.6,
            target_fn=lambda t: (14.0, -8.0 + 0.5 * t),
            walls=tuple((14.0, y, 0.28, 0.22, 1.2) for y in (-2.0, 0.0, 2.0)),
        ),
        Scene(
            key='gust', name='阵风扰动 + 机动目标',
            desc='目标做 0.8 m/s 往复机动，同时叠加幅值约 0.5 m/s² 的阵风',
            duration=22.0, uav0=(4.0, -2.0, P.H_DES), yaw0=0.5,
            target_fn=_line_then_back((12.0, -6.0), (12.0, 6.0), 20.0),
            distractors=((12.0, -6.0),),
            wind_fn=_gust,
        ),
    ]
