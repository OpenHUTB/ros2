#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒立摆（cart-pole）模型、能量摆起与 LQR 平衡控制（纯 Python + numpy）。

坐标与符号约定
--------------
- 状态 ``[x, x_dot, theta, theta_dot]``；
- ``x`` 为小车位置（m），向右为正；
- ``theta`` 为摆杆与**竖直向上方向**的夹角（rad），逆时针为正；
  因此 ``theta = 0`` 是竖直向上（平衡点），``theta = ±pi`` 是自然下垂；
- ``force`` 为作用在小车上的水平力（N），向右为正。

本模块提供：
1. ``cartpole_dynamics``：非线性动力学（供 RK4 积分）；
2. ``total_energy``：相对下垂位置的总机械能；
3. ``lqr_gain``：用离散 Riccati 迭代求 LQR 反馈增益；
4. ``CartPoleController``：能量摆起 + LQR 平衡的双模控制器。
"""
from __future__ import print_function

import math

import numpy as np

# 默认物理参数（与仓库 cartpole_balance 一致）
DEFAULT_PARAMS = {
    'mc': 1.0,        # 小车质量 kg
    'mp': 0.1,        # 摆杆质量 kg
    'l': 0.5,         # 摆杆半长 m
    'g': 9.81,        # 重力加速度
    'force_limit': 10.0,
}

MODE_SWINGUP = 'SWINGUP'
MODE_BALANCE = 'BALANCE'


def wrap_angle(theta):
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(theta), math.cos(theta))


def cartpole_dynamics(state, force, params=None):
    """返回状态导数 [x_dot, x_ddot, theta_dot, theta_ddot]。"""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    mc, mp, l, g = p['mc'], p['mp'], p['l'], p['g']

    x, x_dot, theta, theta_dot = state
    total_mass = mc + mp
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # 线性方程组：
    # [M,        m*l*cos] [x_ddot]   [F + m*l*theta_dot^2*sin]
    # [m*l*cos,  m*l^2  ] [theta_ddot] = [m*g*l*sin]
    a11 = total_mass
    a12 = mp * l * cos_t
    a21 = mp * l * cos_t
    a22 = mp * l * l
    b1 = force + mp * l * theta_dot * theta_dot * sin_t
    b2 = mp * g * l * sin_t

    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-9:
        det = 1e-9
    x_ddot = (b1 * a22 - a12 * b2) / det
    theta_ddot = (a11 * b2 - b1 * a21) / det
    return [x_dot, x_ddot, theta_dot, theta_ddot]


def total_energy(state, params=None):
    """相对自然下垂位置（theta = ±pi，静止）的总机械能（J）。"""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    mp, l, g = p['mp'], p['l'], p['g']
    x_dot, theta, theta_dot = state[1], state[2], state[3]
    kinetic = 0.5 * (p['mc'] + mp) * x_dot * x_dot \
        + mp * l * x_dot * theta_dot * math.cos(theta) \
        + 0.5 * mp * l * l * theta_dot * theta_dot
    potential = mp * g * l * (math.cos(theta) + 1.0)
    return kinetic + potential


def upright_energy(params=None):
    """摆杆静止在竖直向上时的能量，即摆起目标能量。"""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    return 2.0 * p['mp'] * p['g'] * p['l']


def linearize_upright(params=None):
    """在竖直向上平衡点线性化，返回连续系统矩阵 A、B。"""
    p = dict(DEFAULT_PARAMS)
    if params:
        p.update(params)
    mc, mp, l, g = p['mc'], p['mp'], p['l'], p['g']
    total_mass = mc + mp
    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -mp * g / mc, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, total_mass * g / (mc * l), 0.0],
    ])
    B = np.array([[0.0], [1.0 / mc], [0.0], [-1.0 / (mc * l)]])
    return A, B


def lqr_gain(params=None, dt=0.02, q_diag=(1.0, 0.1, 10.0, 0.1), r=1.0,
             iterations=20000, tol=1e-12):
    """离散 Riccati 迭代求 LQR 增益 K（使 u = -K x）。"""
    A, B = linearize_upright(params)
    Ad = np.eye(4) + A * dt
    Bd = B * dt
    Q = np.diag(q_diag)
    R = np.array([[r]])
    P = Q.copy()
    for _ in range(iterations):
        middle = np.linalg.inv(R + Bd.T.dot(P).dot(Bd))
        P_next = Ad.T.dot(P).dot(Ad) - Ad.T.dot(P).dot(Bd).dot(middle).dot(Bd.T).dot(P).dot(Ad) + Q
        if np.max(np.abs(P_next - P)) < tol:
            P = P_next
            break
        P = P_next
    K = np.linalg.inv(R + Bd.T.dot(P).dot(Bd)).dot(Bd.T).dot(P).dot(Ad)
    return K.flatten()


class CartPoleController(object):
    """能量摆起 + LQR 平衡的双模控制器。"""

    def __init__(self, params=None, k_energy=8.0, k_cart_p=0.8, k_cart_d=0.6,
                 switch_angle=0.4, switch_rate=2.0, fall_angle=0.7,
                 balance_dt=0.02, allow_swingup=True):
        self.params = dict(DEFAULT_PARAMS)
        if params:
            self.params.update(params)
        self.force_limit = self.params['force_limit']
        self.allow_swingup = bool(allow_swingup)

        self.k_energy = float(k_energy)
        self.k_cart_p = float(k_cart_p)
        self.k_cart_d = float(k_cart_d)
        self.switch_angle = float(switch_angle)
        self.switch_rate = float(switch_rate)
        self.fall_angle = float(fall_angle)

        self.k_lqr = lqr_gain(self.params, dt=balance_dt)
        self.e_target = upright_energy(self.params)
        self.mode = MODE_SWINGUP if self.allow_swingup else MODE_BALANCE

    def _clamp(self, force):
        return max(-self.force_limit, min(self.force_limit, force))

    def balance_force(self, state):
        return float(-self.k_lqr.dot(np.asarray(state, dtype=float)))

    def swingup_force(self, state):
        x, x_dot, theta, theta_dot = state
        energy = total_energy(state, self.params)
        deficit = self.e_target - energy
        # dE/dt = -m*l*theta_dot*x_ddot*cos(theta)
        # 因此力取 -(theta_dot*cos(theta)) 的符号方向可增加能量
        direction = 1.0 if theta_dot * math.cos(theta) >= 0.0 else -1.0
        force = -self.k_energy * deficit * direction
        force += -self.k_cart_p * x - self.k_cart_d * x_dot
        return self._clamp(force)

    def update(self, state, dt=None):
        """根据当前状态返回控制力（N），并更新模式。"""
        theta = wrap_angle(state[2])
        theta_dot = state[3]

        if not self.allow_swingup:
            return self._clamp(self.balance_force(
                [state[0], state[1], theta, theta_dot]))

        if self.mode == MODE_SWINGUP:
            if abs(theta) < self.switch_angle and abs(theta_dot) < self.switch_rate:
                self.mode = MODE_BALANCE
        else:
            if abs(theta) > self.fall_angle:
                self.mode = MODE_SWINGUP

        if self.mode == MODE_BALANCE:
            return self._clamp(self.balance_force([state[0], state[1], theta, theta_dot]))
        return self.swingup_force([state[0], state[1], theta, theta_dot])
