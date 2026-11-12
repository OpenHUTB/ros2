#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制器模块：前馈 + PID 反馈 + 抗积分饱和 + 感知安全修正。

对应课程任务（2）的控制部分。

本模块为**纯函数 / 纯计算类**，不依赖 ROS。

控制律
------
``v_cmd = ṗ_d + Kp·e + Ki·∫e + Kd·ė``

其中 ``e = p_d - p``。前馈项补偿轨迹自身运动，反馈项消除跟踪误差。

三个容易踩的坑
--------------
1. **积分饱和**：速度饱和时必须冻结积分，否则误差持续累积，
   退出饱和后长时间超调。
2. **感知修正必须是乘性缩放**，不能在阈值处硬切换，
   否则无人机会在阈值附近顿挫。
3. **控制周期必须与仿真步长一致**：本类的增益是按 ``dt`` 整定的，
   改变 ``dt`` 必须重新整定。
"""

from __future__ import annotations

import numpy as np


class TrajectoryController:
    """轨迹跟踪控制器（前馈 + PID + 抗饱和）。"""

    def __init__(self, kp=1.6, ki=0.05, kd=0.60, v_max=3.0, dt=0.1, tau=0.45):
        """
        Parameters
        ----------
        kp, ki, kd : float or array_like
            三个增益。标量会广播到三个轴；也可传长度 3 的数组分别设置。
        v_max : float
            速度饱和上限 (m/s)。
        dt : float
            控制周期 (s)。
        tau : float
            速度环一阶滞后时间常数 (s)。用于前馈相位超前补偿，
            取 0 表示不补偿。
        """
        self.kp = np.broadcast_to(np.asarray(kp, dtype=np.float64), (3,)).copy()
        self.ki = np.broadcast_to(np.asarray(ki, dtype=np.float64), (3,)).copy()
        self.kd = np.broadcast_to(np.asarray(kd, dtype=np.float64), (3,)).copy()
        self.v_max = float(v_max)
        self.dt = float(dt)
        self.tau = float(tau)

        self.integral = np.zeros(3, dtype=np.float64)
        self.prev_error = None
        # 上一周期的参考速度，用于在无预测接口时做一阶外推
        self.prev_pd_dot = None

    def reset(self):
        """清空积分与微分状态（重新开始跟踪时调用）。"""
        self.integral[:] = 0.0
        self.prev_error = None
        self.prev_pd_dot = None

    def feedforward(self, p_d_dot, p_d_dot_next=None):
        """带相位超前的前馈项。

        速度环是一阶滞后 ``τ``，若直接拿当前 ``ṗ_d`` 作前馈，实际速度总是
        **落后** 参考速度约 ``τ`` 秒，产生与速度成正比的结构性跟踪误差
        （实测 5 m 半径 8 字轨迹下约 0.2~0.4 m，且**加大 Kp 无法消除**）。

        补偿办法是让前馈提前 ``τ``：

            ṗ_ff(t) = ṗ_d(t + τ) ≈ ṗ_d(t) + τ · dṗ_d/dt

        其中 ``dṗ_d/dt`` 用相邻两周期的差商近似。若调用方直接传入了
        ``p_d_dot_next``（即 ``ṗ_d(t+dt)``），则用一阶外推得到更准的值。
        """
        p_d_dot = np.asarray(p_d_dot, dtype=np.float64)
        if self.tau <= 0.0:
            self.prev_pd_dot = p_d_dot.copy()
            return p_d_dot

        if p_d_dot_next is not None:
            # 用 (t, t+dt) 两点做一阶外推到 t+tau
            slope = (np.asarray(p_d_dot_next, dtype=np.float64) - p_d_dot) / self.dt
        elif self.prev_pd_dot is not None:
            slope = (p_d_dot - self.prev_pd_dot) / self.dt
        else:
            slope = np.zeros(3, dtype=np.float64)

        self.prev_pd_dot = p_d_dot.copy()
        return p_d_dot + self.tau * slope

    def step(self, p_d, p_d_dot, p, p_d_dot_next=None):
        """推进一步，返回速度指令（未含感知修正）。

        Parameters
        ----------
        p_d : array_like, shape (3,)
            参考位置。
        p_d_dot : array_like, shape (3,)
            参考速度（前馈项，会被相位超前补偿）。
        p : array_like, shape (3,)
            当前实际位置（来自传感器链路）。
        p_d_dot_next : array_like or None
            ``ṗ_d(t + dt)``，可选；提供时前馈外推更准确。

        Returns
        -------
        numpy.ndarray, shape (3,)
            饱和后的速度指令。
        """
        p_d = np.asarray(p_d, dtype=np.float64)
        p_d_dot = np.asarray(p_d_dot, dtype=np.float64)
        p = np.asarray(p, dtype=np.float64)

        e = p_d - p

        # 微分项：首步无历史，取零避免冲击
        if self.prev_error is None:
            de = np.zeros(3, dtype=np.float64)
        else:
            de = (e - self.prev_error) / self.dt
        self.prev_error = e.copy()

        # 试探性累加积分
        self.integral += e * self.dt

        v_ff = self.feedforward(p_d_dot, p_d_dot_next)
        v = v_ff + self.kp * e + self.ki * self.integral + self.kd * de

        # 抗积分饱和：若饱和则回退本次积分，避免误差持续累积
        if np.any(np.abs(v) >= self.v_max):
            self.integral -= e * self.dt

        return np.clip(v, -self.v_max, self.v_max)

    def safety_scale(self, hist, front=(15, 0, 1), s_safe=0.6, s_min=0.25):
        """感知安全系数（委托给 perception.safety_scale，便于单测）。"""
        from perception import safety_scale as _ss
        return _ss(hist, front=front, s_safe=s_safe, s_min=s_min)

    @staticmethod
    def apply_safety(v_cmd, scale):
        """把安全系数乘到速度指令上。scale ∈ [0,1]。"""
        return np.asarray(v_cmd, dtype=np.float64) * float(scale)
