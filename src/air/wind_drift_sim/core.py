#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_drift_sim/core.py - 定常风场下无人机位置漂移仿真核心（ENU 坐标系）

模型（一阶阻力近似, 水平面）:
    无人机悬停时对空气速度为 0, 风速产生相对气流 → 气动阻力
    F = k * (V_wind - V_drone)
    加速度 a = F / m
    有位置反馈补偿时: 控制加速度 = -Kp * pos_error - Kd * vel
对比三种策略的漂移: 无补偿 / 仅速度阻尼 / PD 位置补偿。
"""

import math


class WindDriftPlant:
    """水平面质点 + 风阻模型 (ENU, 单位: m, m/s)。"""

    def __init__(self, wind=(3.0, 0.0), drag_coeff=0.5, mass=1.5,
                 pos=(0.0, 0.0), vel=(0.0, 0.0)):
        self.wind = tuple(map(float, wind))
        self.k = float(drag_coeff)
        self.mass = float(mass)
        self.pos = list(map(float, pos))
        self.vel = list(map(float, vel))

    def step(self, control=(0.0, 0.0), dt=0.02):
        """control: (ax, ay) 控制加速度 (悬停时为 0 = 无补偿)。"""
        for i in range(2):
            drag = self.k * (self.wind[i] - self.vel[i]) / self.mass
            self.vel[i] += (control[i] + drag) * dt
            self.pos[i] += self.vel[i] * dt
        return tuple(self.pos)


def simulate(policy, wind=(3.0, 0.0), duration=30.0, dt=0.02):
    """
    policy(pos, vel, t) -> (ax, ay)
    返回 (时间序列, 位置序列 [(x,y)...])。
    """
    plant = WindDriftPlant(wind=wind)
    ts, xs = [], []
    n = int(duration / dt)
    for i in range(n):
        t = i * dt
        ctrl = policy(plant.pos, plant.vel, t)
        p = plant.step(ctrl, dt)
        ts.append(t)
        xs.append(p)
    return ts, xs


def no_compensation(pos, vel, t):
    return (0.0, 0.0)


def pd_compensation(target=(0.0, 0.0), kp=0.8, kd=1.2):
    def policy(pos, vel, t):
        return tuple(kp * (target[i] - pos[i]) - kd * vel[i] for i in range(2))
    return policy


def drift_metrics(positions):
    """最大漂移距离 (m) 与最终稳态漂移 (m)。"""
    dists = [math.hypot(x, y) for x, y in positions]
    return {'max_drift': max(dists), 'final_drift': dists[-1]}
