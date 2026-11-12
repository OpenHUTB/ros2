"""8 字形轨迹的解析式（纯函数，便于单测）。

曲线：

    x(t) = 2·sin(t)
    y(t) = 2·sin(t)·cos(t) = sin(2t)

航向取轨迹切向 ::

    dx/dt = 2·cos(t)
    dy/dt = 2·cos(2t)
    theta = atan2(dy, dx)

原实现每轮循环把参数 ``t`` 固定加 0.03，动画速度取决于循环实际频率；
这里保留解析式，由调用方按真实时间传入 ``t``。
"""

from __future__ import annotations

import math

AMPLITUDE = 2.0


def position(t: float, amplitude: float = AMPLITUDE) -> tuple[float, float]:
    """返回轨迹点 ``(x, y)``。"""
    t = float(t)
    return amplitude * math.sin(t), amplitude * math.sin(t) * math.cos(t)


def velocity(t: float, amplitude: float = AMPLITUDE) -> tuple[float, float]:
    """返回参数导数 ``(dx/dt, dy/dt)``。

    ``y = A·sin t·cos t = (A/2)·sin 2t``，所以 ``dy/dt = A·cos 2t``。
    """
    t = float(t)
    return amplitude * math.cos(t), amplitude * math.cos(2.0 * t)


def heading(t: float, amplitude: float = AMPLITUDE) -> float:
    """切向航向角，范围 ``(-π, π]``。"""
    dx, dy = velocity(t, amplitude)
    return math.atan2(dy, dx)


def quaternion(t: float, amplitude: float = AMPLITUDE) -> tuple[float, float, float, float]:
    """绕 z 轴航向角的四元数 ``(x, y, z, w)``。"""
    theta = heading(t, amplitude)
    return 0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)
