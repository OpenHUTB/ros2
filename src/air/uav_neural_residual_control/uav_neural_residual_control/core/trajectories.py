#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参考轨迹：解析的位置/速度/加速度/偏航，供几何控制器做前馈。

全部写成闭式解而不是数值微分：``a_d`` 要直接进推力分配，差分噪声会以
``m·a`` 的量级变成推力抖动（0.95 kg 上 0.1 m/s² 的差分噪声就是 0.1 N）。

坐标约定与底座一致：世界系 ENU（x 东、y 北、z 上）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

__all__ = ["Trajectory", "Hover", "Circle", "Helix", "Figure8", "TRAJECTORIES",
           "make_trajectory"]


@dataclass
class Trajectory:
    """轨迹基类：``eval(t)`` 返回 (pos, vel, acc, yaw, yaw_rate)。"""

    name: str = "traj"
    duration: float = 12.0

    def eval(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        raise NotImplementedError


# --------------------------------------------------------------------------- #
class Hover(Trajectory):
    """定点悬停 —— 最严苛的稳态误差测试：没有任何前馈运动掩盖模型偏差。"""

    def __init__(self, pos=(0.0, 0.0, 1.5), yaw: float = 0.0, duration: float = 10.0):
        super().__init__(name="hover", duration=duration)
        self.pos = np.asarray(pos, dtype=float)
        self.yaw = float(yaw)

    def eval(self, t):
        return self.pos.copy(), np.zeros(3), np.zeros(3), self.yaw, 0.0


# --------------------------------------------------------------------------- #
class Circle(Trajectory):
    """定高圆：向心加速度恒定，检验阻力在**径向**的稳态偏差。"""

    def __init__(self, radius: float = 2.0, height: float = 1.5,
                 period: float = 10.0, turns: int = 1, face_tangent: bool = True):
        super().__init__(name="circle", duration=period * turns)
        self.r = float(radius)
        self.z = float(height)
        self.w = 2.0 * math.pi / float(period)
        self.face_tangent = bool(face_tangent)

    def eval(self, t):
        w, r = self.w, self.r
        c, s = math.cos(w * t), math.sin(w * t)
        pos = np.array([r * c, r * s, self.z])
        vel = np.array([-r * w * s, r * w * c, 0.0])
        acc = np.array([-r * w * w * c, -r * w * w * s, 0.0])
        yaw = w * t + (math.pi / 2.0 if self.face_tangent else 0.0)
        return pos, vel, acc, yaw, w


# --------------------------------------------------------------------------- #
class Helix(Trajectory):
    """螺旋爬升：垂直方向持续有速度，直接暴露推力标定误差（载荷）。
    载荷每多 100 g，标称模型就少 0.245 N/桨，等速爬升会退化成下沉。

    起始高度不能太低：控制器按 0.95 kg 标称质量给推力，挂 0.75 kg 时会以
    约 4.3 m/s² 下坠，PD 稳住前的动态下冲约为静态静差（0.4 m）的 1.5 倍。
    从 0.8 m 起飞的实测结果是直接触地翻机（倾角 177°），从 1.5 m 起则
    全程离地，残差网络与 RLS 都来得及把下坠拉回来。
    """

    def __init__(self, radius: float = 2.0, z0: float = 1.5, z1: float = 3.5,
                 period: float = 8.0, turns: float = 2.0):
        super().__init__(name="helix", duration=period * turns)
        self.r = float(radius)
        self.z0, self.z1 = float(z0), float(z1)
        self.w = 2.0 * math.pi / float(period)
        self.vz = (self.z1 - self.z0) / self.duration

    def eval(self, t):
        w, r = self.w, self.r
        c, s = math.cos(w * t), math.sin(w * t)
        pos = np.array([r * c, r * s, self.z0 + self.vz * t])
        vel = np.array([-r * w * s, r * w * c, self.vz])
        acc = np.array([-r * w * w * c, -r * w * w * s, 0.0])
        return pos, vel, acc, w * t + math.pi / 2.0, w


# --------------------------------------------------------------------------- #
class Figure8(Trajectory):
    """Gerono 双纽线，机头始终朝速度方向。

    过原点时速度最大而曲率变号，是**大加速度 + 快速姿态反转**的组合，
    残差补偿慢一拍就会在这里甩出最大误差。
    """

    def __init__(self, ax: float = 2.5, ay: float = 1.4, height: float = 1.8,
                 period: float = 12.0, loops: int = 1):
        super().__init__(name="figure8", duration=period * loops)
        self.ax, self.ay = float(ax), float(ay)
        self.z = float(height)
        self.w = 2.0 * math.pi / float(period)

    def eval(self, t):
        w, ax, ay = self.w, self.ax, self.ay
        s1, c1 = math.sin(w * t), math.cos(w * t)
        s2, c2 = math.sin(2 * w * t), math.cos(2 * w * t)
        pos = np.array([ax * s1, ay * s2, self.z])
        vel = np.array([ax * w * c1, 2 * ay * w * c2, 0.0])
        acc = np.array([-ax * w * w * s1, -4 * ay * w * w * s2, 0.0])
        yaw = math.atan2(vel[1], vel[0])
        # 航向角速度由几何关系给出，避免再对 atan2 做一次差分
        speed2 = float(vel[0] ** 2 + vel[1] ** 2) + 1e-12
        yaw_rate = (vel[0] * acc[1] - vel[1] * acc[0]) / speed2
        return pos, vel, acc, yaw, yaw_rate


# --------------------------------------------------------------------------- #
TRAJECTORIES: Dict[str, type] = {
    "hover": Hover,
    "circle": Circle,
    "helix": Helix,
    "figure8": Figure8,
}

#: 论文式对比实验用的两条代表轨迹：一条慢而平滑，一条快而急变
BENCH_TRAJECTORIES = ("helix", "figure8")


def make_trajectory(name: str, **kwargs) -> Trajectory:
    if name not in TRAJECTORIES:
        raise KeyError("未知轨迹 %r，可选 %s" % (name, list(TRAJECTORIES)))
    return TRAJECTORIES[name](**kwargs)
