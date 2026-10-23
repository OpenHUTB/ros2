#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车辆模型：运动学自行车模型（默认）与线性二自由度动力学模型。

两者的状态都是 ``[x, y, yaw, v]``，但**横向自由度模型不同**：

* 运动学模型：车辆无侧滑，航向角速度直接由前轮转角决定，适合低速与教学演示。
* 动力学模型：引入质心侧偏角与横摆角速度两个状态，存在轮胎侧偏刚度、
  高速时会表现不足转向/过度转向，适合检验控制器在中高速下的鲁棒性。

两种模型对控制器暴露同一接口 ``step(state, delta, accel)``，因此三个控制器
无需感知底层模型差异，可在同一份代码下做交叉对比（这是本模块评测公平性的基础）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class VehicleParams:
    """车辆几何与动力学参数（默认值取自小型乘用车的典型量级）。"""

    wheelbase: float = 2.7        # 轴距 L (m)
    cg_to_front: float = 1.2      # 质心到前轴 a (m)
    cg_to_rear: float = 1.5       # 质心到后轴 b (m)，a+b 应等于 wheelbase
    mass: float = 1500.0          # 质量 (kg)，仅动力学模型使用
    yaw_inertia: float = 2000.0   # 横摆转动惯量 Iz (kg·m^2)
    cf: float = 90000.0           # 前轮侧偏刚度 (N/rad)
    cr: float = 110000.0          # 后轮侧偏刚度 (N/rad)
    max_steering: float = math.radians(35.0)   # 前轮最大转角
    max_accel: float = 2.0        # 纵向加速度上限 (m/s^2)
    max_decel: float = 4.0        # 纵向减速度上限 (m/s^2)
    # 是否允许倒车（v < 0）。默认 False：本模块的横向控制器与指标
    # （横向偏差、航向偏差、收敛时间）都是按"前进"定义的，
    # 倒车时这些量没有同等含义，因此默认把 v 钳在 0 并记录被拦截的步数。
    allow_reverse: bool = False

    def __post_init__(self) -> None:
        if abs(self.cg_to_front + self.cg_to_rear - self.wheelbase) > 1e-6:
            # 允许略有不一致，但给出明确提示，避免 LQR 线性化用错车长
            self.wheelbase = self.cg_to_front + self.cg_to_rear
        # 边界情况显式报错，避免负值被静默钳位成 0 后"看起来正常"
        if not math.isfinite(self.max_accel) or self.max_accel < 0:
            raise ValueError(f"max_accel 必须是非负数，当前 {self.max_accel}")
        if not math.isfinite(self.max_decel) or self.max_decel < 0:
            raise ValueError(f"max_decel 必须是非负数，当前 {self.max_decel}")
        if not math.isfinite(self.max_steering) or self.max_steering <= 0:
            raise ValueError(f"max_steering 必须为正数，当前 {self.max_steering}")


class KinematicBicycle:
    """运动学自行车模型（后轴参考点）。

    状态 [x, y, yaw, v]，控制量 [delta(前轮转角), accel(纵向加速度)]::

        x'   = v * cos(yaw)
        y'   = v * sin(yaw)
        yaw' = v * tan(delta) / L
        v'   = accel

    采用 RK4 积分，步长 0.02 s 时与解析解误差可忽略。
    """

    name = "kinematic"

    def __init__(self, params: VehicleParams | None = None) -> None:
        self.p = params or VehicleParams()
        # 一次仿真中被"不允许倒车"规则拦截的步数（0 表示从未触发）
        self.reverse_blocked_steps = 0

    def _effective_speed(self, v: float) -> float:
        """供积分使用的前进速度。

        不允许倒车时把负速度当作 0 参与位置/航向积分。只在步末钳位是不够的：
        RK4 的中间级（k2/k3/k4）会取到负速度，导致车虽然"停下"却仍有微小后向位移。
        """
        if v < 0.0 and not self.p.allow_reverse:
            return 0.0
        return float(v)

    def derivative(self, state: np.ndarray, delta: float, accel: float) -> np.ndarray:
        x, y, yaw, v = state
        v_eff = self._effective_speed(v)
        return np.array([
            v_eff * math.cos(yaw),
            v_eff * math.sin(yaw),
            v_eff * math.tan(delta) / self.p.wheelbase,
            accel,
        ])

    def step(self, state: np.ndarray, delta: float, accel: float, dt: float) -> np.ndarray:
        delta = float(np.clip(delta, -self.p.max_steering, self.p.max_steering))
        accel = float(np.clip(accel, -self.p.max_decel, self.p.max_accel))
        k1 = self.derivative(state, delta, accel)
        k2 = self.derivative(state + 0.5 * dt * k1, delta, accel)
        k3 = self.derivative(state + 0.5 * dt * k2, delta, accel)
        k4 = self.derivative(state + dt * k3, delta, accel)
        new_state = state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        new_state[3] = self._resolve_speed(new_state[3])
        return new_state

    def _resolve_speed(self, v_new: float) -> float:
        """处理积分后可能出现负速度的边界情况。

        * ``allow_reverse=True``：保留负速度，即真实倒车；
        * ``allow_reverse=False``（默认）：把速度钳到 0 并累加
          ``reverse_blocked_steps``，使这一行为可观测、可断言，
          而不是静默截断——否则调用方会误以为车辆在正常前进。
        """
        if v_new < 0.0 and not self.p.allow_reverse:
            self.reverse_blocked_steps += 1
            return 0.0
        return float(v_new)

    def lateral_limits(self, v: float) -> float:
        """当前车速下能实现的最大横摆角速度（供控制器判断可行性）。

        取 ``|v|``：倒车（``allow_reverse=True``）时曲线半径与前进同向、
        角速度反号，但可行域大小相同。
        """
        v_abs = abs(float(v))
        if v_abs < 1e-6:
            return 0.0
        return v_abs * math.tan(self.p.max_steering) / self.p.wheelbase


class DynamicBicycle:
    """线性二自由度动力学模型（质心参考点，含轮胎侧偏刚度）。

    状态扩展为 ``[x, y, yaw, v, vy, r]`` 中对外仍只暴露 ``[x, y, yaw, v]``，
    内部另存侧向速度 vy 与横摆角速度 r。线性化方程::

        vy' = -(cf + cr)/(m*v) * vy - (a*cf - b*cr)/(m*v) * r + cf/m * delta - v*r
        r'  = -(a*cf - b*cr)/(Iz*v) * vy - (a^2*cf + b^2*cr)/(Iz*v) * r + a*cf/Iz * delta

    当 v 很小时上面两式会出现除零，故低于 ``v_min`` 时退化为运动学模型，
    这在实际控制器切换中也是常规做法。
    """

    name = "dynamic"

    def __init__(self, params: VehicleParams | None = None, v_min: float = 2.0) -> None:
        self.p = params or VehicleParams()
        self.v_min = float(v_min)
        self.kinematic = KinematicBicycle(self.p)
        self.reverse_blocked_steps = 0

    def derivative(self, state: np.ndarray, delta: float, accel: float,
                   vy: float, r: float) -> tuple:
        x, y, yaw, v = state
        p = self.p
        if v < self.v_min:
            d = self.kinematic.derivative(state, delta, accel)
            yaw_rate = d[2]
            vy_dot = 0.0
            r_dot = 0.0
            return d, vy_dot, r_dot
        cf, cr, m, iz = p.cf, p.cr, p.mass, p.yaw_inertia
        a, b = p.cg_to_front, p.cg_to_rear
        vy_dot = (-(cf + cr) / (m * v) * vy
                  - (a * cf - b * cr) / (m * v) * r
                  + cf / m * delta - v * r)
        r_dot = (-(a * cf - b * cr) / (iz * v) * vy
                 - (a * a * cf + b * b * cr) / (iz * v) * r
                 + a * cf / iz * delta)
        return np.array([
            v * math.cos(yaw) - vy * math.sin(yaw),
            v * math.sin(yaw) + vy * math.cos(yaw),
            r,
            accel,
        ]), vy_dot, r_dot

    def step(self, state: np.ndarray, delta: float, accel: float, dt: float,
             memory: dict | None = None) -> tuple:
        """返回 (新状态 [x,y,yaw,v], 新 memory)。"""
        delta = float(np.clip(delta, -self.p.max_steering, self.p.max_steering))
        accel = float(np.clip(accel, -self.p.max_decel, self.p.max_accel))
        mem = dict(memory or {"vy": 0.0, "r": 0.0})
        vy, r = mem["vy"], mem["r"]

        d1, vy1, r1 = self.derivative(state, delta, accel, vy, r)
        d2, vy2, r2 = self.derivative(state + 0.5 * dt * d1, delta, accel,
                                      vy + 0.5 * dt * vy1, r + 0.5 * dt * r1)
        d3, vy3, r3 = self.derivative(state + 0.5 * dt * d2, delta, accel,
                                      vy + 0.5 * dt * vy2, r + 0.5 * dt * r2)
        d4, vy4, r4 = self.derivative(state + dt * d3, delta, accel,
                                      vy + dt * vy3, r + dt * r3)
        new_state = state + dt / 6.0 * (d1 + 2 * d2 + 2 * d3 + d4)
        new_state[3] = self.kinematic._resolve_speed(new_state[3])
        self.reverse_blocked_steps = self.kinematic.reverse_blocked_steps
        new_mem = {
            "vy": float(vy + dt / 6.0 * (vy1 + 2 * vy2 + 2 * vy3 + vy4)),
            "r": float(r + dt / 6.0 * (r1 + 2 * r2 + 2 * r3 + r4)),
        }
        return new_state, new_mem
