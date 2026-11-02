#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_altitude_controller/core.py - PID 高度控制器与无人机高度动力学仿真核心

模型（简化二阶模型）:
    推力加速度 a = throttle * max_accel - g   （throttle ∈ [0,1] 限幅）
    垂直速度 vz += a * dt
    高度     z  += vz * dt
PID 控制器根据高度误差计算 throttle，验证阶跃响应性能
（超调量、调节时间、稳态误差）。
"""

import math

G = 9.81


class PID:

    def __init__(self, kp, ki, kd, output_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement
        self.integral += error * dt
        deriv = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * deriv
        # 抗积分饱和: 输出限幅时停止积分累加
        if abs(output) > self.output_limit and error * self.integral > 0:
            self.integral -= error * dt
        return max(-self.output_limit, min(self.output_limit, output))


class AltitudePlant:
    """无人机垂直方向二阶动力学。"""

    def __init__(self, max_accel=16.0, z0=0.0):
        # 满节流阀加速度 16 m/s^2 > g=9.81, 悬停节流阀约 0.61
        self.max_accel = max_accel
        self.z = z0
        self.vz = 0.0

    def step(self, throttle: float, dt: float):
        accel = throttle * self.max_accel - G
        self.vz += accel * dt
        self.z += self.vz * dt
        return self.z


def simulate_altitude_step(kp, ki, kd, target_alt=10.0, duration=20.0, dt=0.02):
    """高度阶跃仿真, 返回 (时间序列, 高度序列, 节流阀序列, 性能指标字典)。"""
    pid = PID(kp, ki, kd, output_limit=1.0)
    plant = AltitudePlant()
    ts, zs, us = [], [], []
    t = 0.0
    while t <= duration:
        u = pid.update(target_alt, plant.z, dt)
        plant.step(max(0.0, u), dt)  # 节流阀非负
        ts.append(t)
        zs.append(plant.z)
        us.append(u)
        t += dt

    # 性能指标
    peak = max(zs)
    overshoot = max(0.0, (peak - target_alt) / target_alt * 100.0)
    settle_idx = None
    for i in range(len(zs) - 1, -1, -1):
        if abs(zs[i] - target_alt) > 0.05 * target_alt:
            settle_idx = i + 1
            break
    settling_time = ts[settle_idx] if settle_idx is not None and settle_idx < len(ts) else ts[-1]
    steady_err = abs(zs[-1] - target_alt)
    metrics = {'peak': peak, 'overshoot_pct': overshoot,
               'settling_time': settling_time, 'steady_error': steady_err}
    return ts, zs, us, metrics
