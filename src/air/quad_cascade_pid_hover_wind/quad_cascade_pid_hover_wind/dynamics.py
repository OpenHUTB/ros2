#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""6 自由度四旋翼刚体动力学 + 电机一阶响应 + 风扰模型。

机体坐标系：x 前、y 左、z 上；世界系 ENU。
四旋翼为 X 型布置：
  FR (前右) 顺时针, FL (前左) 逆时针,
  RL (后左) 顺时针, RR (后右) 逆时针。
"""

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from .geometry import (body_to_world, quat_derivative, quat_normalize,
                       quat_to_rotation, world_to_body)


@dataclass
class QuadParams:
    mass: float = 1.2
    arm: float = 0.18
    ixx: float = 0.012
    iyy: float = 0.012
    izz: float = 0.022
    k_f: float = 3.2e-6
    k_tau: float = 6.5e-8
    min_omega: float = 200.0
    max_omega: float = 1100.0
    motor_tau: float = 0.04
    gravity: float = 9.81


@dataclass
class WindParams:
    mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gust_magnitude: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gust_start: float = 18.0
    gust_duration: float = 6.0
    turbulence_sigma: float = 0.4
    turbulence_bandwidth: float = 1.0
    drag_coeff: float = 0.35
    area: float = 0.12
    air_density: float = 1.225


@dataclass
class State:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))
    omega_cmd: np.ndarray = field(default_factory=lambda: np.zeros(4))  # 4 个旋翼指令转速

    def copy(self):
        return State(
            pos=self.pos.copy(), vel=self.vel.copy(), quat=self.quat.copy(),
            omega=self.omega.copy(), omega_cmd=self.omega_cmd.copy())


class Quadrotor:
    """X 型四旋翼。"""

    # 旋翼相对机身中心的位置 [FR, FL, RL, RR]
    ROTOR_POS = np.array([
        [1.0, -1.0, 0.0],   # FR
        [1.0, 1.0, 0.0],    # FL
        [-1.0, 1.0, 0.0],   # RL
        [-1.0, -1.0, 0.0],  # RR
    ])
    # 每个旋翼的自旋方向：+1 逆时针，-1 顺时针（俯视）
    SPIN_DIR = np.array([-1.0, 1.0, -1.0, 1.0])

    def __init__(self, params: QuadParams, wind: WindParams):
        self.p = params
        self.w = wind
        self.inertia = np.diag([params.ixx, params.iyy, params.izz])
        self.inertia_inv = np.diag([1.0 / params.ixx, 1.0 / params.iyy, 1.0 / params.izz])
        # 混合矩阵：将 (总推力, 滚转力矩, 俯仰力矩, 偏航力矩) 映射到 4 个旋翼推力
        self._mixer = self._build_mixer()
        self._turbulence_state = np.zeros(3)

    def _build_mixer(self) -> np.ndarray:
        """输入 u = [T, tau_x, tau_y, tau_z]^T，输出 4 个旋翼推力 T_i。"""
        L = self.p.arm
        c = self.p.k_tau / self.p.k_f
        # T1(FR)..T4(RR) -> [T, tau_x, tau_y, tau_z] 的映射
        # tau_x = L*(left - right), tau_y = L*(rear - front),
        # tau_z = c*(CCW - CW)
        A = np.array([
            [1.0, 1.0, 1.0, 1.0],
            L * np.array([-1.0, 1.0, 1.0, -1.0]),
            L * np.array([-1.0, -1.0, 1.0, 1.0]),
            c * np.array([-1.0, 1.0, -1.0, 1.0]),
        ])
        return np.linalg.inv(A)

    def forces_and_moments(self, state: State, wind_vel: np.ndarray
                           ) -> Tuple[np.ndarray, np.ndarray]:
        """计算当前状态下的合外力（世界系）与合力矩（机体系）。"""
        p = self.p
        omega = np.clip(state.omega_cmd, p.min_omega, p.max_omega)
        thrust = p.k_f * omega ** 2  # 每个旋翼的推力（沿机体 +z）
        reaction = p.k_tau * omega ** 2 * self.SPIN_DIR  # 反扭矩（绕机体 z）

        total_thrust = np.array([0.0, 0.0, np.sum(thrust)])  # 机体系
        force_body = total_thrust.copy()

        # 力矩（机体系）：向量化计算 sum(r_i × [0, 0, T_i])
        r = self.ROTOR_POS * p.arm
        thrust_vecs = np.column_stack([np.zeros(4), np.zeros(4), thrust])
        tau_rotor = np.sum(np.cross(r, thrust_vecs), axis=0)
        tau_yaw = np.array([0.0, 0.0, np.sum(reaction)])
        moment_body = tau_rotor + tau_yaw

        # 重力（世界系，负 z）
        force_world = np.array([0.0, 0.0, -p.mass * p.gravity])
        # 推力转到世界系
        force_world += body_to_world(state.quat, force_body)

        # 气动阻力（世界系）：相对速度 = 机体速度 - 风速
        v_rel = state.vel - wind_vel
        v_rel_norm = np.linalg.norm(v_rel)
        if v_rel_norm > 1e-6:
            drag = -0.5 * self.w.air_density * self.w.drag_coeff * self.w.area \
                * v_rel_norm * v_rel
            force_world += drag

        return force_world, moment_body

    def wind_velocity(self, t: float, dt: float) -> np.ndarray:
        """世界系风速，含恒定风、阵风（半正弦包络）与一阶湍流。"""
        v = np.array(self.w.mean, dtype=float)
        # 阵风（半正弦包络）
        if self.w.gust_start <= t <= self.w.gust_start + self.w.gust_duration:
            phase = math.pi * (t - self.w.gust_start) / max(self.w.gust_duration, 1e-6)
            v += np.array(self.w.gust_magnitude) * math.sin(phase)
        # 一阶低通白噪声作为湍流
        bw = max(self.w.turbulence_bandwidth, 1e-6)
        sigma = self.w.turbulence_sigma
        noise = np.random.randn(3) * sigma * math.sqrt(2.0 * bw * dt)
        self._turbulence_state += -bw * self._turbulence_state * dt + noise
        v += self._turbulence_state
        return v

    def step(self, state: State, omega_cmd: np.ndarray, dt: float,
             t: float) -> Tuple[State, np.ndarray]:
        """积分一步，返回新状态与世界系风速。"""
        p = self.p
        s = state.copy()

        # 电机一阶响应
        alpha = dt / max(p.motor_tau, 1e-6)
        alpha = min(alpha, 1.0)
        s.omega_cmd = s.omega_cmd + alpha * (
            np.clip(omega_cmd, p.min_omega, p.max_omega) - s.omega_cmd)

        wind = self.wind_velocity(t, dt)
        force_world, moment_body = self.forces_and_moments(s, wind)

        # 平动
        acc_world = force_world / p.mass
        s.vel = s.vel + acc_world * dt
        s.pos = s.pos + s.vel * dt

        # 转动（机体系）
        omega = s.omega
        tau = moment_body - np.cross(omega, self.inertia @ omega)
        alpha_body = self.inertia_inv @ tau
        s.omega = omega + alpha_body * dt

        # 四元数积分
        qdot = quat_derivative(s.quat, s.omega)
        s.quat = quat_normalize(s.quat + qdot * dt)

        return s, wind


def mixer_matrix(params: QuadParams) -> np.ndarray:
    """外部可直接获取的混合矩阵。"""
    q = Quadrotor(params, WindParams())
    return q._mixer.copy()


def thrust_to_omega(thrust: np.ndarray, k_f: float, min_omega: float, max_omega: float) -> np.ndarray:
    """推力 -> 转速。"""
    safe = np.clip(thrust, 0.0, None)
    omega = np.sqrt(safe / max(k_f, 1e-12))
    return np.clip(omega, min_omega, max_omega)
