#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""姿态数学、混控与几何控制器（无 mujoco 依赖）。

底座把这三样和 ``Quadrotor`` 放在同一个模块里，而 ``Quadrotor`` 顶层
``import mujoco``；ROS 侧是 Python 3.8、装不了 mujoco 3.12，一 import 就崩。
把纯数学部分独立出来，仿真侧与 ROS 侧共用同一份实现，仿真里整定好的增益
在 ROS 上行为一致。

数值上与 ``uav_mujoco_sim.core.quadrotor`` 的同名实现逐行等价；
``--selftest`` 里有一致性校验，防止两边日后走偏。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

__all__ = ["GRAVITY", "QuadParams", "QuadrotorState", "quat_to_mat", "mat_to_quat",
           "quat_to_euler", "vec_to_skew", "vee", "mixer", "GeometricController"]

GRAVITY = 9.81


# --------------------------------------------------------------------------- #
# 姿态工具
# --------------------------------------------------------------------------- #
def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """四元数 (w,x,y,z) → 3x3 旋转矩阵（机体→世界）。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → 四元数 (w,x,y,z)（数值稳定版）。"""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw)，ZYX 欧拉角，单位弧度。"""
    R = quat_to_mat(q)
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)


def vec_to_skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def vee(M: np.ndarray) -> np.ndarray:
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) * 0.5


# --------------------------------------------------------------------------- #
# 机体参数与状态
# --------------------------------------------------------------------------- #
@dataclass
class QuadParams:
    """控制器眼中的机体参数（单位：kg / m / N·m·s² / N）。"""

    mass: float
    arm: float           # 单桨到机心的 x/y 分量：a = L/(2√2)
    k_m: float           # 反扭矩系数：τz = k_m·(−T1 − T2 + T3 + T4)
    max_thrust: float    # 单桨最大推力
    hover_thrust: float  # 单桨悬停推力 = m·g/4

    @property
    def gravity(self) -> float:
        return GRAVITY


@dataclass
class QuadrotorState:
    """一帧机体状态（位置/速度世界系 ENU，角速度机体系）。"""

    t: float
    pos: np.ndarray
    quat: np.ndarray
    vel: np.ndarray
    omega: np.ndarray
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    rotor_thrust: np.ndarray = field(default_factory=lambda: np.zeros(4))

    @classmethod
    def from_odom(cls, t, pos, quat, vel, omega) -> "QuadrotorState":
        roll, pitch, yaw = quat_to_euler(np.asarray(quat, float))
        return cls(t=float(t), pos=np.asarray(pos, float), quat=np.asarray(quat, float),
                   vel=np.asarray(vel, float), omega=np.asarray(omega, float),
                   roll=roll, pitch=pitch, yaw=yaw)


# --------------------------------------------------------------------------- #
# 混控
# --------------------------------------------------------------------------- #
def mixer(T: float, tau: np.ndarray, p: QuadParams) -> np.ndarray:
    """期望合推力 T 与力矩 tau=[τx,τy,τz] → 四个旋翼推力。

    u = τx/a, v = τy/a, w = τz/k_m，四桨推力形如 (T ± u ± v ± w)/4，
    全部落在 [0, T_max] 的充要条件：|u| + |v| + |w| ≤ min(4·T_max − T, T)。

    饱和时必须整体等比缩小 (u,v,w)、不能逐个 clip —— 逐个 clip 会破坏力矩
    方向，底座实测圆形轨迹误差从 17 mm 发散到 474 m。
    """
    u = tau[0] / p.arm
    v = tau[1] / p.arm
    w = tau[2] / p.k_m

    T = float(np.clip(T, 0.0, 4.0 * p.max_thrust))
    budget = min(4.0 * p.max_thrust - T, T)
    spread = abs(u) + abs(v) + abs(w)
    if spread > budget and spread > 1e-12:
        k = budget / spread
        u, v, w = u * k, v * k, w * k

    out = np.array([
        (T - u - v - w) * 0.25,
        (T + u + v - w) * 0.25,
        (T + u + w - v) * 0.25,
        (T + v + w - u) * 0.25,
    ])
    return np.clip(out, 0.0, p.max_thrust)


# --------------------------------------------------------------------------- #
# 几何控制器（外环位置 + 内环姿态，一次算完）
# --------------------------------------------------------------------------- #
class GeometricController:
    """SE(3) 几何控制器：位置/速度/加速度/偏航指令 → 合推力与力矩。

    外环 f_des = m·(a_d + Kp⊙(p−p_d) + Kd⊙(v−v_d) + g·e_z)，
    由 f_des 的方向与期望偏航构造 R_des；内环用几何误差
    e_R = ½·vee(R_desᵀR − RᵀR_des) 做 PD。

    增益符号沿用底座约定：误差取 ``当前值 − 期望值``，故增益为负。
    """

    def __init__(self, params: QuadParams,
                 kp=(-16.0, -16.0, -20.0), kd=(-8.0, -8.0, -9.0),
                 kr=(-1.5, -1.5, -0.6), kw=(-0.20, -0.20, -0.14)):
        self.p = params
        self.kp = np.asarray(kp, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.kr = np.asarray(kr, dtype=float)
        self.kw = np.asarray(kw, dtype=float)
        self.max_tilt = np.deg2rad(35.0)   # 限倾角，防大误差时翻跟头

    def compute(self, st: QuadrotorState, p_d, v_d, a_d, yaw_d
                ) -> Tuple[float, np.ndarray]:
        """返回 (合推力 T, 机体力矩 [τx,τy,τz])。"""
        e_p = st.pos - np.asarray(p_d, float)
        e_v = st.vel - np.asarray(v_d, float)

        acc = np.asarray(a_d, float) + self.kp * e_p + self.kd * e_v
        f = self.p.mass * (acc + np.array([0.0, 0.0, GRAVITY]))
        # 限倾角：水平分量不超过 tan(max_tilt) × 垂直分量
        fz = max(f[2], 0.15 * self.p.mass * GRAVITY)
        fh = float(np.linalg.norm(f[:2]))
        lim = float(np.tan(self.max_tilt) * fz)
        if fh > lim and fh > 1e-9:
            f[:2] *= lim / fh

        z_b = f / max(float(np.linalg.norm(f)), 1e-9)
        x_c = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
        y_b = np.cross(z_b, x_c)
        n = float(np.linalg.norm(y_b))
        if n < 1e-6:                       # 期望偏航与推力方向共线时的退化处理
            y_b = np.array([-np.sin(yaw_d), np.cos(yaw_d), 0.0])
        else:
            y_b = y_b / n
        x_b = np.cross(y_b, z_b)
        R_des = np.column_stack([x_b, y_b, z_b])

        R = quat_to_mat(st.quat)
        e_R = vee(R_des.T @ R - R.T @ R_des)
        tau_b = self.kr * e_R + self.kw * st.omega
        T = float(np.dot(f, R[:, 2]))      # 合推力沿机体 z 的投影
        return T, tau_b

    def act(self, st: QuadrotorState, p_d, v_d, a_d, yaw_d,
            params: QuadParams = None) -> np.ndarray:
        """一步：状态 → 力矩 → 逆混控 → 四桨推力。"""
        T, tau = self.compute(st, p_d, v_d, a_d, yaw_d)
        return mixer(T, tau, params if params is not None else self.p)
