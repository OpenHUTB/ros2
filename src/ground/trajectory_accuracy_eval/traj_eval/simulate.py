#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成"真值轨迹 + 带误差里程计轨迹"，用于离线演示与回归测试。

里程计误差模型贴近真实的轮式里程计标定问题：

* ``wheel_scale``：轮半径标定误差 → 距离尺度误差（跑得越远偏得越多）；
* ``wheelbase_scale``：轮距标定误差 → 转角尺度误差；
* ``yaw_bias_deg_per_m``：两轮差动/陀螺零偏导致的航向缓慢累积偏差；
* ``trans_noise_std`` / ``yaw_noise_std``：单步随机噪声。

由此得到的漂移是**系统性误差**主导的（真实里程计的主要误差来源），
正好可以用本模块的指标量化"标定前后差多少"。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from .se2 import Pose, compose, inverse, wrap_angle


@dataclass
class OdomErrorModel:
    """里程计误差模型参数。"""

    wheel_scale: float = 1.0          # 距离尺度（1.02 = 每 100 m 多算 2 m）
    wheelbase_scale: float = 1.0      # 转角尺度
    yaw_bias_deg_per_m: float = 0.0   # 每米航向偏置（度）
    trans_noise_std: float = 0.0      # 单步平移噪声标准差 (m)
    yaw_noise_std_deg: float = 0.0    # 单步航向噪声标准差 (度)
    seed: int = 0


@dataclass
class TrajectoryKind:
    """内置真值轨迹的类型参数。"""

    name: str
    curvature_amp: float = 0.0
    curvature_bias: float = 0.0
    curvature_period: float = 20.0


KINDS = {
    "straight": TrajectoryKind("straight", 0.0, 0.0, 20.0),
    "circle": TrajectoryKind("circle", 0.0, 0.1, 20.0),
    "figure8": TrajectoryKind("figure8", 0.12, 0.0, 16.0),
    "sine": TrajectoryKind("sine", 0.06, 0.0, 24.0),
}


def make_ground_truth(kind: str = "figure8", speed: float = 1.5,
                      duration: float = 40.0, dt: float = 0.05) -> List[Pose]:
    """用单轨模型积分生成真值轨迹。

    ``kappa(t) = curvature_bias + curvature_amp * sin(2*pi*t/period)``，
    恒定车速 ``speed``。
    """
    if kind not in KINDS:
        raise KeyError(f"未知轨迹类型 '{kind}'，可选: {', '.join(KINDS)}")
    if speed <= 0 or dt <= 0:
        raise ValueError("speed 与 dt 必须为正")
    k = KINDS[kind]

    poses: List[Pose] = [Pose(0.0, 0.0, 0.0)]
    t = 0.0
    while t < duration - 1e-9:
        kappa = k.curvature_bias + k.curvature_amp * math.sin(2.0 * math.pi * t / k.curvature_period)
        rel = Pose(speed * dt * math.cos(0.5 * kappa * speed * dt),
                   speed * dt * math.sin(0.5 * kappa * speed * dt),
                   wrap_angle(kappa * speed * dt))
        poses.append(compose(poses[-1], rel))
        t += dt
    return poses


def simulate_odometry(gt: Sequence[Pose], model: OdomErrorModel | None = None) -> List[Pose]:
    """按误差模型从真值"积分"出一条里程计轨迹。

    每一步取真值的相对运动，套上尺度/偏置/噪声误差后重新积分，
    因此误差会随行驶距离累积——这正是真实轮式里程计的行为。
    """
    if len(gt) < 2:
        raise ValueError("真值轨迹至少需要 2 个位姿")
    m = model or OdomErrorModel()
    rng = np.random.default_rng(m.seed)

    yaw_bias = math.radians(m.yaw_bias_deg_per_m)
    yaw_noise = math.radians(m.yaw_noise_std_deg)

    odom: List[Pose] = [gt[0]]
    for i in range(len(gt) - 1):
        rel = compose(inverse(gt[i]), gt[i + 1])
        ds = math.hypot(rel.x, rel.y)
        dtheta = rel.yaw

        ds_meas = ds * m.wheel_scale
        if m.trans_noise_std > 0:
            ds_meas += float(rng.normal(0.0, m.trans_noise_std))
        dtheta_meas = dtheta * m.wheelbase_scale + yaw_bias * ds
        if yaw_noise > 0:
            dtheta_meas += float(rng.normal(0.0, yaw_noise))

        # 差速里程计只观测到"前进距离 + 转角"
        odom.append(compose(odom[-1], Pose(ds_meas, 0.0, wrap_angle(dtheta_meas))))
    return odom


def default_error_models() -> dict:
    """返回几个典型误差模型，便于做"标定前后"对比。"""
    return {
        "perfect": OdomErrorModel(),
        "wheel_2pct": OdomErrorModel(wheel_scale=1.02),
        "calibrated": OdomErrorModel(wheel_scale=1.002),
        "yaw_bias": OdomErrorModel(yaw_bias_deg_per_m=0.1),
        "noisy": OdomErrorModel(trans_noise_std=0.002, yaw_noise_std_deg=0.05, seed=7),
    }
