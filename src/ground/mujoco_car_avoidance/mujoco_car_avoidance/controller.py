# -*- coding: utf-8 -*-
"""速度规划与差速运动学（纯函数，不导入 MuJoCo）。

分两层：
    1. SpeedRamp —— 对决策给出的目标速度做加速度/减速度限幅，
       避免"目标速度瞬间归零"造成的急刹点头，也让"减速度"成为可评价指标；
    2. differential_drive —— 把 (线速度, 角速度) 换算成左右轮目标角速度。
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RampParams:
    """加减速限幅。"""

    acc_max: float = 2.0     #: 最大加速度 (m/s^2)
    brake_max: float = 3.5   #: 最大减速度 (m/s^2)，取正值


class SpeedRamp:
    """把阶跃的目标速度整形为加减速受限的速度指令。"""

    def __init__(self, params: RampParams = RampParams()):
        self.params = params
        self.speed = 0.0

    def reset(self, speed: float = 0.0) -> None:
        self.speed = float(speed)

    def update(self, target_speed: float, dt: float) -> float:
        """按 |dv| <= a*dt 逼近目标速度，返回整形后的速度指令。"""
        if dt <= 0.0:
            return self.speed

        target_speed = max(0.0, float(target_speed))
        delta = target_speed - self.speed

        if delta >= 0.0:
            limit = self.params.acc_max * dt
        else:
            limit = self.params.brake_max * dt

        if abs(delta) <= limit:
            self.speed = target_speed
        else:
            self.speed += math.copysign(limit, delta)
        return self.speed


def differential_drive(linear_mps: float,
                       angular_radps: float,
                       wheel_radius_m: float,
                       wheel_separation_m: float,
                       max_wheel_omega: float = float("inf")):
    """差速运动学：(线速度, 角速度) -> (左轮角速度, 右轮角速度)。

    v_left  = v - omega * L / 2
    v_right = v + omega * L / 2
    omega_wheel = v_wheel / r

    返回 (left_omega, right_omega, saturated)，saturated 表示是否因超出轮速上限被限幅。
    """
    half = wheel_separation_m / 2.0
    v_left = linear_mps - angular_radps * half
    v_right = linear_mps + angular_radps * half

    left = v_left / wheel_radius_m
    right = v_right / wheel_radius_m

    saturated = False
    if max_wheel_omega != float("inf"):
        for value in (left, right):
            if abs(value) > max_wheel_omega:
                saturated = True
        left = max(-max_wheel_omega, min(max_wheel_omega, left))
        right = max(-max_wheel_omega, min(max_wheel_omega, right))

    return left, right, saturated


def emergency_stop(ramp: SpeedRamp) -> float:
    """把速度斜坡直接清零（用于急停 / STALE 兜底），返回零速。"""
    ramp.reset(0.0)
    return 0.0
