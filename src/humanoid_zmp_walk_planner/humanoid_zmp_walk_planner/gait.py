"""简化双足步态的解析轨迹（纯函数，便于单元测试）。

约定
----
* ``t`` 为仿真时间 [s]，``step_time`` 为一个完整步态周期（左右各一步）[s]；
* ``body_x`` 在一个周期内前进 **一个** ``step_length``（修正了原先
  ``x = t * step_length`` 把周期位移放大 1/step_time 倍的问题）；
* ``foot_heights`` 返回左右脚相对地面的抬升高度与当前支撑脚：
  前半周期左脚支撑、右脚摆动，后半周期相反；
  摆动脚高度按 ``step_height * sin(2π·相位)`` 抬落，起落点均为 0。
"""

from __future__ import annotations

import math

PHASE_SWITCH = 0.5


def phase(t: float, step_time: float) -> float:
    """周期内相位，取值 ``[0, 1)``。"""
    if step_time <= 0.0:
        raise ValueError("step_time 必须 > 0")
    return (float(t) % float(step_time)) / float(step_time)


def body_x(t: float, step_length: float, step_time: float) -> float:
    """躯干沿 +x 的位移：每个完整周期（左右各一步）前进 ``2 * step_length``。

    ``step_length`` 是**单步**长度（一只脚迈出的距离），一个周期迈两步。
    """
    if step_time <= 0.0:
        raise ValueError("step_time 必须 > 0")
    return 2.0 * float(step_length) * float(t) / float(step_time)


def foot_heights(phase_value: float, step_height: float) -> tuple[float, float, str]:
    """按相位返回 ``(左脚高度, 右脚高度, 支撑脚)``。

    支撑脚高度恒为 0；摆动脚在相位 0.5（或 0/1）处高度为 0，峰值 ``step_height``。
    """
    p = float(phase_value)
    if not 0.0 <= p < 1.0:
        raise ValueError("phase 必须在 [0, 1)")
    if p < PHASE_SWITCH:
        left = 0.0
        right = step_height * math.sin(p / PHASE_SWITCH * math.pi)
        return left, right, "left"
    left = step_height * math.sin((p - PHASE_SWITCH) / PHASE_SWITCH * math.pi)
    right = 0.0
    return left, right, "right"


def leg_targets(t: float, step_length: float, step_time: float,
                step_height: float) -> tuple[float, float, float, float, str]:
    """一次算齐动画需要的量：``(body_x, 左脚高, 右脚高, 相位, 支撑脚)``。"""
    p = phase(t, step_time)
    left, right, support = foot_heights(p, step_height)
    return body_x(t, step_length, step_time), left, right, p, support
