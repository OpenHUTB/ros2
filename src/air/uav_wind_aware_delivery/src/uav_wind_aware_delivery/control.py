# -*- coding: utf-8 -*-
"""简化几何控制器：位置/速度 -> 世界系合力 + 世界系力矩。

不使用旋翼混控，直接对机体施加合力和力矩；姿态内环让机体跟随合力方向。
这样可以把项目重点放在“风扰落点预测与释放补偿”，同时保留可见的倾斜飞行。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .mathutil import quat_to_mat, vee
from .state import State


@dataclass
class ControlConfig:
    kp: float = 5.5
    kd: float = 4.2
    ki: float = 0.9
    kr: float = 4.0
    kw: float = 0.85
    max_acc: float = 18.0
    max_tilt_deg: float = 32.0
    max_torque: float = 1.2
    integral_limit: float = 0.9


class GeometricController:
    def __init__(self, cfg: ControlConfig | None = None):
        self.cfg = cfg or ControlConfig()
        self.integral = np.zeros(3)

    def compute(self, st: State, p_d: np.ndarray, v_d: np.ndarray,
                yaw_d: float = 0.0, mass: float = 1.17):
        c = self.cfg
        p_d = np.asarray(p_d, dtype=float)
        v_d = np.asarray(v_d, dtype=float)

        # 期望加速度（PID）+ 重力补偿；积分项消除恒定风扰下的稳态误差
        e_p = st.pos - p_d
        self.integral += e_p * 0.01
        self.integral = np.clip(self.integral, -c.integral_limit, c.integral_limit)
        a = -c.kp * e_p - c.kd * (st.vel - v_d) - c.ki * self.integral
        n = float(np.linalg.norm(a))
        if n > c.max_acc:
            a *= c.max_acc / n
        f = mass * (a + np.array([0.0, 0.0, 9.81]))
        fmax = 2.6 * mass * 9.81
        nf = float(np.linalg.norm(f))
        if nf > fmax:
            f *= fmax / nf

        # 限倾角：水平力不超过 tan(max_tilt) * 垂直力
        fz = max(float(f[2]), 0.25 * mass * 9.81)
        fh = float(np.linalg.norm(f[:2]))
        lim = np.tan(np.deg2rad(c.max_tilt_deg)) * fz
        if fh > lim and fh > 1e-9:
            f[:2] *= lim / fh
            fz = max(float(f[2]), 0.25 * mass * 9.81)

        # 由合力方向和期望偏航构造期望姿态
        z_b = f / max(float(np.linalg.norm(f)), 1e-9)
        x_c = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
        y_b = np.cross(z_b, x_c)
        ny = float(np.linalg.norm(y_b))
        if ny < 1e-6:
            y_b = np.array([-np.sin(yaw_d), np.cos(yaw_d), 0.0])
        else:
            y_b /= ny
        x_b = np.cross(y_b, z_b)
        R_des = np.column_stack([x_b, y_b, z_b])

        R = quat_to_mat(st.quat)
        e_R = vee(R_des.T @ R - R.T @ R_des)
        tau_b = -c.kr * e_R - c.kw * st.omega
        nt = float(np.linalg.norm(tau_b))
        if nt > c.max_torque:
            tau_b *= c.max_torque / nt
        tau_w = R @ tau_b
        return f, tau_w

    def act(self, sim, p_d, v_d, yaw_d: float = 0.0):
        st = sim.state()
        f, tau = self.compute(st, p_d, v_d, yaw_d, mass=sim.mass)
        sim.step(f, tau)
        return f, tau
