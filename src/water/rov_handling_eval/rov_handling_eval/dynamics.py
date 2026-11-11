# -*- coding: utf-8 -*-
"""纯 numpy 的水动力计算（不依赖 ROS / MuJoCo，便于单元测试）。

坐标系约定
----------
* 世界系：**z 轴向上**（与 MuJoCo 默认、ROS ENU 一致），深度 = −z。
  实机上常用 NED（z 向下），本模块为了和仿真内核/ROS 对齐特意不翻符号，
  只在发布 `/rov/depth` 时取负——把符号转换压缩到一处，是这个模块的设计原则。
* 机体系：FLU，x 前、y 左、z 上（REP-103）。

单位：长度 m、质量 kg、力 N、力矩 N·m、密度 kg/m³。
"""

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

G = 9.81


@dataclass
class VehicleParams:
    """载具与水动力参数。默认值按 BlueROV2 Heavy 量级取。"""

    mass: float = 11.0                 # 干重 kg
    displacement: float = 0.0110       # 排水体积 m³（略大于 m/ρ，形成正浮力）
    rho: float = 1025.0                # 海水密度
    ballast: float = 0.0               # 额外配重 kg（正 = 更重）

    # 各轴等效二次阻力系数 k = 0.5·ρ·Cd·A，单位 N·s²/m²
    drag_linear: Sequence[float] = field(default_factory=lambda: (13.20, 35.69, 63.33))
    # 角阻尼（线性近似，N·m·s/rad）
    drag_angular: Sequence[float] = field(default_factory=lambda: (0.60, 0.60, 0.40))

    # 浮心相对质心的位置（机体系，通常在上方 → 提供横滚/俯仰恢复力矩）
    cb_offset: Sequence[float] = field(default_factory=lambda: (0.0, 0.0, 0.03))

    # 推进器可用推力上限（每轴合力，N）与力矩上限
    max_force: Sequence[float] = field(default_factory=lambda: (10.0, 10.0, 10.0))
    max_torque: Sequence[float] = field(default_factory=lambda: (2.0, 2.0, 2.0))

    def k(self) -> np.ndarray:
        return np.asarray(self.drag_linear, dtype=float)

    def buoyancy(self) -> float:
        """浮力 B = ρ·∇·g（N，向上）。"""
        return self.rho * self.displacement * G

    def weight(self) -> float:
        """总重（含配重）W = (m + Δm)·g（N，向下）。"""
        return (self.mass + self.ballast) * G

    def net_buoyancy(self) -> float:
        """净浮力（向上为正）。0 表示配平，正数表示会自己上浮。"""
        return self.buoyancy() - self.weight()

    def terminal_speed(self, axis: int = 2) -> float:
        """只有净浮力、没有推力时，该轴上的漂移终端速度（m/s）。

        v_t = sqrt(|F_net| / k)，由 ½·ρ·Cd·A·v² = F 解出。
        """
        k = float(self.k()[axis])
        return math.sqrt(abs(self.net_buoyancy()) / k) if k > 0 else 0.0


def drag_force(v_rel: np.ndarray, k: np.ndarray) -> np.ndarray:
    """二次阻力：F = −k·v·|v|（永不做负功）。"""
    v = np.asarray(v_rel, dtype=float)
    kk = np.asarray(k, dtype=float)
    return -kk * v * np.abs(v)


def drag_torque(w: np.ndarray, k_rot: Sequence[float]) -> np.ndarray:
    """角阻尼（线性近似足够做手操手感）。"""
    return -np.asarray(k_rot, dtype=float) * np.asarray(w, dtype=float)


def buoyancy_wrench(params: VehicleParams, R=None) -> tuple:
    """浮力 + 重力的合力与合力矩（世界系，z 向上）。

    浮心在质心上方 `cb_offset` 时，浮力会产生恢复力矩
    τ = r × F，这正是 ROV 被推一下会自己回正的原因；如果只把
    浮力/重力各自相减成一个力加在质心，横滚/俯仰就会失去自稳性。

    参数
    ----
    R : 3×3，机体系 → 世界系；省略时按单位阵（用于离线测试）。
    """
    R = np.eye(3) if R is None else np.asarray(R, dtype=float).reshape(3, 3)
    f_buoy_world = np.array([0.0, 0.0, params.buoyancy()])
    f_world = np.array([0.0, 0.0, params.net_buoyancy()])
    r_world = R @ np.asarray(params.cb_offset, dtype=float)   # 浮心相对质心（世界系）
    tau = np.cross(r_world, f_buoy_world)
    return f_world, tau


def velocity_control(cmd_body: np.ndarray, v_body: np.ndarray, w_body: np.ndarray,
                     params: VehicleParams, kp_lin: float = 60.0,
                     kp_ang: float = 8.0) -> tuple:
    """速度环 → 期望的机体力/力矩（N, N·m），已限幅。

    这是"手操"的核心：操作员给的是速度意图，力由控制器换算。
    开环（直接力映射）会把推力器的非线性直接暴露给手感。
    """
    cmd = np.asarray(cmd_body, dtype=float).reshape(6)
    f = kp_lin * (cmd[:3] - np.asarray(v_body, dtype=float))
    tau = kp_ang * (cmd[3:] - np.asarray(w_body, dtype=float))
    f = np.clip(f, -np.asarray(params.max_force, dtype=float),
                np.asarray(params.max_force, dtype=float))
    tau = np.clip(tau, -np.asarray(params.max_torque, dtype=float),
                  np.asarray(params.max_torque, dtype=float))
    return f, tau


def current_authority(params: VehicleParams, current_speed: float) -> np.ndarray:
    """抗流"操控权限利用率" η（每轴一个值，0～1）。

    η = (可用推力 − 顶流所需推力) / 可用推力

    η<0.2 时几乎操纵不动，这是判定"这个流速下还能不能干活"的依据。
    顶流所需推力按"横向均匀流"估算：F_need = k_i · v_c²（各轴同流速）。
    """
    avail = np.asarray(params.max_force, dtype=float)
    need = np.asarray(params.k(), dtype=float) * float(current_speed) ** 2
    eta = (avail - need) / np.maximum(avail, 1e-9)
    return np.clip(eta, 0.0, 1.0)
