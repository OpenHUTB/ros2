#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四旋翼 6 自由度刚体动力学（离线可运行，无 ROS 依赖）。

坐标系约定：
    机体系：x 向前，y 向左，z 向上；
    世界系：ENU（东 x、北 y、上 z）。

四旋翼为 X 型布置，四个旋翼编号：
    FR 前右（顺时针）、FL 前左（逆时针）、
    RL 后左（顺时针）、RR 后右（逆时针）。

本模块只做刚体平动/转动积分与旋翼推力-力矩混合分配，
不包含闭环航点控制；键盘遥控指令由 teleop 模块映射为本模块的
``cmd`` 指令字典。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np


@dataclass
class QuadParams:
    """四旋翼物理参数。

    属性:
        mass: 整机质量 kg。
        arm: 旋翼中心到机身中心的力臂 m。
        ixx / iyy / izz: 三轴转动惯量 kg·m²。
        k_f: 推力系数，推力 T = k_f · ω²。
        k_tau: 反扭矩系数，反扭矩 τ = k_tau · ω²。
        min_omega / max_omega: 旋翼转速下限/上限 rad/s。
        motor_tau: 电机一阶响应时间常数 s。
        gravity: 重力加速度 m/s²。
        drag_coeff: 气动阻力系数（无量纲）。
        area: 迎风面积 m²。
        air_density: 空气密度 kg/m³。
    """

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
    drag_coeff: float = 0.35
    area: float = 0.12
    air_density: float = 1.225
    rot_damping: float = 0.06
    att_kp: float = 4.0
    att_kd: float = 0.15


@dataclass
class State:
    """四旋翼状态。"""

    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw
    angular_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    omega_cmd: np.ndarray = field(default_factory=lambda: np.zeros(4))

    def copy(self) -> "State":
        return State(
            pos=self.pos.copy(),
            vel=self.vel.copy(),
            rpy=self.rpy.copy(),
            angular_vel=self.angular_vel.copy(),
            omega_cmd=self.omega_cmd.copy(),
        )


# 旋翼相对机身中心的位置 [FR, FL, RL, RR]，单位：力臂倍数
ROTOR_POS = np.array([
    [1.0, -1.0, 0.0],
    [1.0, 1.0, 0.0],
    [-1.0, 1.0, 0.0],
    [-1.0, -1.0, 0.0],
])
# 自旋方向：+1 逆时针（俯视），-1 顺时针
SPIN_DIR = np.array([-1.0, 1.0, -1.0, 1.0])


def normalize_angle(a: float) -> float:
    """把角度归一化到 (-π, π]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Quadrotor:
    """X 型四旋翼刚体动力学仿真器。"""

    def __init__(self, params: QuadParams | None = None):
        self.p = params or QuadParams()
        self.inertia = np.diag([self.p.ixx, self.p.iyy, self.p.izz])
        self.inertia_inv = np.diag([
            1.0 / self.p.ixx, 1.0 / self.p.iyy, 1.0 / self.p.izz,
        ])
        self._mixer = self._build_mixer()

    # ------------------------------------------------------------------ #
    # 混合分配
    # ------------------------------------------------------------------ #
    def _build_mixer(self) -> np.ndarray:
        """构造混合矩阵 A，满足 [T, τx, τy, τz]^T = A · [T1..T4]^T。

        逆混合矩阵 M = A^{-1} 把期望合力/合力矩映射到四个旋翼推力。
        """
        L = self.p.arm
        c = self.p.k_tau / self.p.k_f
        A = np.array([
            [1.0, 1.0, 1.0, 1.0],
            L * np.array([-1.0, 1.0, 1.0, -1.0]),
            L * np.array([-1.0, -1.0, 1.0, 1.0]),
            c * np.array([-1.0, 1.0, -1.0, 1.0]),
        ])
        return np.linalg.inv(A)

    def hover_thrust(self) -> float:
        """悬停所需总推力（N），等于整机重力 mg。"""
        return self.p.mass * self.p.gravity

    def thrust_to_omega(self, thrust: np.ndarray) -> np.ndarray:
        """旋翼推力 -> 转速，并限制在 [min_omega, max_omega]。"""
        safe = np.clip(thrust, 0.0, None)
        omega = np.sqrt(safe / max(self.p.k_f, 1e-12))
        return np.clip(omega, self.p.min_omega, self.p.max_omega)

    def allocate(self, thrust_total: float, tau_x: float, tau_y: float,
                tau_z: float) -> np.ndarray:
        """期望合力/力矩 -> 四个旋翼推力。

        参数:
            thrust_total: 期望总推力（机体系 z，N）。
            tau_x: 期望滚转力矩（机体系 x，N·m）。
            tau_y: 期望俯仰力矩（机体系 y，N·m）。
            tau_z: 期望偏航力矩（机体系 z，N·m）。

        返回:
            长度 4 的旋翼推力数组 [FR, FL, RL, RR]。
        """
        wrench = np.array([thrust_total, tau_x, tau_y, tau_z])
        thrusts = self._mixer @ wrench
        return np.clip(thrusts, 0.0, None)

    # ------------------------------------------------------------------ #
    # 动力学积分
    # ------------------------------------------------------------------ #
    def forces_and_moments(self, state: State) -> Tuple[np.ndarray, np.ndarray]:
        """计算当前状态下的世界系合外力与机体系合力矩。"""
        p = self.p
        omega = np.clip(state.omega_cmd, p.min_omega, p.max_omega)
        thrust = p.k_f * omega ** 2
        reaction = p.k_tau * omega ** 2 * SPIN_DIR

        # 机体系推力（沿 +z）
        force_body = np.array([0.0, 0.0, float(np.sum(thrust))])

        # 力矩：旋翼推力对中心的力矩 + 旋翼反扭矩
        r = ROTOR_POS * p.arm
        thrust_vecs = np.column_stack([np.zeros(4), np.zeros(4), thrust])
        tau_rotor = np.sum(np.cross(r, thrust_vecs), axis=0)
        tau_yaw = np.array([0.0, 0.0, float(np.sum(reaction))])
        moment_body = tau_rotor + tau_yaw

        # 重力（世界系，-z）
        force_world = np.array([0.0, 0.0, -p.mass * p.gravity])

        # 推力转到世界系（小角度近似：按当前 rpy 构造旋转矩阵）
        force_world += self._rotate_body_to_world(force_body, state.rpy)

        # 线性气动阻力：F_d = -0.5 ρ Cd A |v| v
        v = state.vel
        v_norm = float(np.linalg.norm(v))
        if v_norm > 1e-6:
            drag = -0.5 * p.air_density * p.drag_coeff * p.area * v_norm * v
            force_world += drag

        return force_world, moment_body

    @staticmethod
    def _rotate_body_to_world(vec: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """小角度近似下把机体系向量旋转到世界系（ZYX 欧拉角）。"""
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        # R = Rz(yaw) · Ry(pitch) · Rx(roll)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        rot = rz @ ry @ rx
        return rot @ vec

    def step(self, state: State, wrench: dict, dt: float) -> State:
        """推进一个仿真步。

        参数:
            state: 当前状态。
            wrench: 合力/合力矩指令字典，字段：
                thrust: 总推力（机体系 z，N）。
                tau_x / tau_y / tau_z: 三轴力矩（机体系，N·m）。
            dt: 仿真步长 s。

        返回:
            推进后的新状态（不修改原状态）。
        """
        p = self.p
        s = state.copy()

        # 电机一阶响应：ω_cmd 跟踪目标转速
        alpha = min(dt / max(p.motor_tau, 1e-6), 1.0)
        target_thrust = self.allocate(
            wrench["thrust"], wrench["tau_x"],
            wrench["tau_y"], wrench["tau_z"])
        target_omega = self.thrust_to_omega(target_thrust)
        s.omega_cmd = s.omega_cmd + alpha * (target_omega - s.omega_cmd)

        force_world, moment_body = self.forces_and_moments(s)

        # 平动积分
        acc_world = force_world / p.mass
        s.vel = s.vel + acc_world * dt
        s.pos = s.pos + s.vel * dt

        # 姿态稳定内环：姿态角回正力矩 + 角速度阻尼（等效虚拟稳定环）
        omega = s.angular_vel
        restore = -self.p.att_kp * s.rpy - self.p.att_kd * omega
        moment_body = moment_body + restore

        # 转动积分：ω̇ = I^{-1}(M - ω × Iω - 旋转阻尼)
        gyro = np.cross(omega, self.inertia @ omega)
        damping = self.p.rot_damping * omega
        alpha_body = self.inertia_inv @ (moment_body - gyro - damping)
        s.angular_vel = omega + alpha_body * dt

        # 欧拉角速率积分（小角度近似）
        rates = self._angular_vel_to_euler_rates(s.angular_vel, s.rpy)
        s.rpy = s.rpy + rates * dt
        s.rpy = np.array([normalize_angle(a) for a in s.rpy])

        # 地面约束：不穿地
        if s.pos[2] <= 0.0:
            s.pos[2] = 0.0
            if s.vel[2] < 0.0:
                s.vel[2] = 0.0

        return s

    def _max_moment(self) -> float:
        """单个通道可用的最大力矩（N·m），用于力矩指令归一化。"""
        return self.p.arm * self.p.k_f * self.p.max_omega ** 2

    @staticmethod
    def _angular_vel_to_euler_rates(omega: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """机体系角速度 -> 欧拉角速率（小角度近似下解耦）。"""
        roll, pitch, _ = rpy
        # 小角度下：φ̇ ≈ ωx，θ̇ ≈ ωy，ψ̇ ≈ ωz
        return np.array([omega[0], omega[1], omega[2]])


def validate_params(params: QuadParams) -> list[str]:
    """校验物理参数合法性，返回错误信息列表（空列表表示通过）。"""
    errors: list[str] = []
    if params.mass <= 0:
        errors.append("mass 必须大于 0")
    if params.arm <= 0:
        errors.append("arm 必须大于 0")
    if params.k_f <= 0:
        errors.append("k_f 必须大于 0")
    if params.min_omega < 0 or params.max_omega <= params.min_omega:
        errors.append("转速范围需满足 0 <= min_omega < max_omega")
    if params.gravity <= 0:
        errors.append("gravity 必须大于 0")
    return errors
