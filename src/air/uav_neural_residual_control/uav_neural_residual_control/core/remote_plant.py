#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""远程机体代理：把"从里程计读状态、向仿真写下推力"伪装成机体对象。

作用只有一个 —— 让 :class:`~.controllers.Tracker` 与 :class:`~.core.plant.ResidualPlant`
之间的鸭子类型接口在**网络那一侧**同样成立。于是同一份控制器代码，
既可以进程内直接驱动仿真，也可以跨 TCP 驱动，不需要第二套实现。
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .attitude import GRAVITY, QuadParams, QuadrotorState, quat_to_mat

__all__ = ["RemotePlant"]


class RemotePlant:
    """只依赖可观测量（里程计 + 自己下达的推力）的机体替身。

    它**不知道**真实质量、真实阻力，也不需要知道：残差本来就是要靠观测反推的。
    """

    def __init__(self, nominal_mass: float = 0.95, dt: float = 0.01,
                 max_thrust: float = 6.0, arm: float = 0.106,
                 k_m: float = 0.015):
        self.dt = float(dt)
        self.nominal_params = QuadParams(
            mass=float(nominal_mass), arm=float(arm), k_m=float(k_m),
            max_thrust=float(max_thrust),
            hover_thrust=float(nominal_mass) * GRAVITY / 4.0)
        self._state: Optional[QuadrotorState] = None
        self._thrust_cmd = np.zeros(4)
        self._quat_prev = np.array([1.0, 0.0, 0.0, 0.0])
        self._vel_prev = np.zeros(3)
        self._last_acc = np.zeros(3)
        self._have_prev = False

    # ------------------------------------------------------------ 接入
    def update(self, t: float, pos, quat, vel, omega) -> None:
        """喂入一帧里程计（每控制周期调用一次）。"""
        pos = np.asarray(pos, dtype=float)
        quat = np.asarray(quat, dtype=float)
        vel = np.asarray(vel, dtype=float)
        if self._have_prev:
            self._last_acc = (vel - self._vel_prev) / self.dt
        else:
            self._have_prev = True
        self._quat_prev = np.asarray(self._state.quat, dtype=float) if self._state \
            else quat.copy()
        self._vel_prev = vel.copy()
        self._state = QuadrotorState.from_odom(t, pos, quat, vel, omega)

    def set_thrust(self, thrusts) -> None:
        """记录本周期下达的推力（残差基准要用它，不是实测值）。"""
        self._thrust_cmd = np.asarray(thrusts, dtype=float).copy()

    # ------------------------------------------------------------ Tracker 接口
    def state(self) -> QuadrotorState:
        if self._state is None:
            raise RuntimeError("尚未收到里程计，先调用 update()")
        return self._state

    @property
    def last_acc(self) -> np.ndarray:
        return self._last_acc.copy()

    @property
    def thrust_cmd(self) -> np.ndarray:
        return self._thrust_cmd.copy()

    @property
    def thrust_applied(self) -> np.ndarray:
        return self._thrust_cmd.copy()      # 网络那侧看不到饱和，如实返回指令

    def nominal_accel(self, thrusts=None, quat=None) -> np.ndarray:
        q = self._quat_prev if quat is None else quat
        t = self._thrust_cmd if thrusts is None else np.asarray(thrusts, dtype=float)
        R = quat_to_mat(q)
        return R @ np.array([0.0, 0.0, float(np.sum(t)) / self.nominal_params.mass]) \
            + np.array([0.0, 0.0, -GRAVITY])

    @property
    def params(self) -> QuadParams:
        """真值参数在网络上不可得，用标称值兜底（只有饱和判定会用到）。"""
        return self.nominal_params
