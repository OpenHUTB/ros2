# -*- coding: utf-8 -*-
"""纵向控制器：把目标速度转成油门 / 刹车（纯 Python，不依赖 CARLA）。

CARLA 的车辆控制接口吃的是 ``throttle ∈ [0, 1]`` 和 ``brake ∈ [0, 1]``，
不是直接给加速度。所以需要一个转换层：

    v_err = v_target - v_current
    a_cmd = kp·v_err + ki·∫v_err            （PI）
    a_cmd 与 0 比较 → 正数给油门，负数给刹车
    油门/刹车量与 |a_cmd| 成正比，并限幅到 [0, 1]

这样既能跟踪目标速度，也能在 v_target = 0 时刹停。

为什么不用纯 P
--------------
有坡度或阻力时纯 P 会有稳态误差（永远差一点速度追不上），
加个积分项就能补掉。积分项带限幅，避免长时间停车后积分饱和、
起步时猛地窜出去。
"""

from __future__ import annotations

__all__ = ["LongitudinalController", "ControllerState"]


class ControllerState(object):
    """控制器的内部状态，便于仿真里回放/记录。"""

    def __init__(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.a_cmd = 0.0


class LongitudinalController(object):
    """PI 纵向速度控制器。

    参数
    ----
    kp           比例增益 (1/s)
    ki           积分增益 (1/s²)
    integral_max 积分项限幅，防止饱和
    throttle_gain 加速度 → 油门的换算系数
    brake_gain    减速度 → 刹车的换算系数
    deadband      速度误差死区 (m/s)，避免抖动
    """

    def __init__(self, kp=0.8, ki=0.15, integral_max=3.0,
                 throttle_gain=0.35, brake_gain=0.5, deadband=0.05):
        if kp < 0:
            raise ValueError("kp 不能为负")
        if ki < 0:
            raise ValueError("ki 不能为负")
        if integral_max < 0:
            raise ValueError("integral_max 不能为负")
        if throttle_gain <= 0 or brake_gain <= 0:
            raise ValueError("增益必须为正")
        if deadband < 0:
            raise ValueError("deadband 不能为负")

        self.kp = float(kp)
        self.ki = float(ki)
        self.integral_max = float(integral_max)
        self.throttle_gain = float(throttle_gain)
        self.brake_gain = float(brake_gain)
        self.deadband = float(deadband)
        self.state = ControllerState()

    # -------------------------------------------------------------------
    def reset(self):
        self.state = ControllerState()

    def update(self, v_current, v_target, dt, a_feedforward=0.0):
        """推进一个控制周期，返回 (throttle, brake)。

        v_current       当前速度 (m/s)
        v_target        目标速度 (m/s)
        dt              控制周期 (s)
        a_feedforward   前馈加速度 (m/s²)，负值表示预期要减速

        为什么需要前馈
        --------------
        停车曲线 v = sqrt(2·a·d) 本身就规定了所需减速度恰好是 a。只靠 PI 的话，
        速度一接近目标误差就变小，刹车随之变弱，最后往往差一点没停住（实测
        会冲过停止线）。把 -a 作为前馈加进去，误差项只需要修正偏差，
        跟踪精度立刻上来。
        """
        if dt <= 0:
            raise ValueError("dt 必须为正")

        v_current = max(0.0, float(v_current))
        v_target = max(0.0, float(v_target))

        err = v_target - v_current
        if abs(err) < self.deadband:
            err = 0.0

        # 积分（带限幅）
        self.state.integral += err * dt
        if self.state.integral > self.integral_max:
            self.state.integral = self.integral_max
        elif self.state.integral < -self.integral_max:
            self.state.integral = -self.integral_max

        a_cmd = (float(a_feedforward)
                 + self.kp * err
                 + self.ki * self.state.integral)
        self.state.a_cmd = a_cmd
        self.state.last_error = err

        throttle = 0.0
        brake = 0.0
        if a_cmd >= 0.0:
            throttle = min(1.0, a_cmd * self.throttle_gain)
        else:
            brake = min(1.0, -a_cmd * self.brake_gain)

        # 目标为 0 且已经很慢时直接踩死，避免积分慢慢拖
        if v_target <= 1e-9 and v_current < 0.3:
            throttle = 0.0
            brake = 1.0
            self.state.integral = 0.0

        self.state.throttle = throttle
        self.state.brake = brake
        return throttle, brake
