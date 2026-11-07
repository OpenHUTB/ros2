#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三种吊挂负载控制律（同一套位置环，区别只在参考/反馈）：

1. ``PdController``  —— 纯位置 PD（无积分项），不感知负载摆动（基线）；
2. ``ZvController``  —— 位置环 + ZV 输入整形，把阶跃拆成相隔半个摆动周期的两段；
3. ``SwingController``—— 位置环 + 负载摆角/速度反馈（“追着负载走”的阻尼项）。

位置环刻意不加积分项：吊挂摆的等效干扰是持续存在的低频耦合，积分项会在摆的
周期附近累积相位滞后，容易把系统推向极限环；稳态精度由姿态环内环与地面
无扰动假设兜底，代价是存在厘米级稳态误差（见评测表中的位置 RMS）。

三者共用 ``_attitude_and_mixer()``：位置环给出期望加速度，换算成期望姿态，
再用 PD 姿态环 + 旋翼混控得到四个推力。因此对比只改变“如何使用负载状态”。
"""

import math

import numpy as np

from sim_core import (ARM, INERTIA, K_M, MAX_THRUST, SWING_PERIOD)


class PdController:
    name = "pd"

    def __init__(self, kp_pos=0.5, kd_pos=1.4, kp_att=25.0, kd_att=7.0,
                 kp_yaw=8.0, kd_yaw=2.5, max_tilt_deg=25.0):
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.kp_att = kp_att
        self.kd_att = kd_att
        self.kp_yaw = kp_yaw
        self.kd_yaw = kd_yaw
        self.max_tilt = math.radians(max_tilt_deg)

    def reset(self):
        pass

    # ---------------------------------------------------------------- 控制入口
    def reference(self, sim, target):
        """子类在这里改写参考轨迹（默认直接跟踪目标）。"""
        return np.asarray(target, dtype=float)

    def accel(self, sim, ref):
        """位置环：期望加速度。子类可叠加摆角反馈。"""
        return self.kp_pos * (ref - sim.base_pos) - self.kd_pos * sim.base_vel

    def command(self, sim, target):
        ref = self.reference(sim, target)
        a_d = self.accel(sim, ref)
        return self._attitude_and_mixer(sim, self._horizontal_limit(a_d))

    def _horizontal_limit(self, a_d):
        a_x, a_y = float(a_d[0]), float(a_d[1])
        norm = math.hypot(a_x, a_y)
        limit = 9.81 * math.tan(self.max_tilt)
        if norm > limit:
            a_x, a_y = a_x * limit / norm, a_y * limit / norm
        return np.array([a_x, a_y, float(a_d[2])])

    # ----------------------------------------------------------- 姿态环 + 混控
    def _attitude_and_mixer(self, sim, a_d):
        g = 9.81
        roll, pitch, yaw = sim.euler
        w = sim.omega
        mass = sim.total_mass

        # 期望推力方向 → 期望滚转/俯仰（机体系 FLU：俯仰正 → +x，滚转正 → -y）
        az_total = a_d[2] + g
        pitch_d = math.atan2(a_d[0], az_total)
        roll_d = math.atan2(-a_d[1], az_total)
        e_roll = roll_d - roll
        e_pitch = pitch_d - pitch
        e_yaw = -yaw

        tau_x = INERTIA[0] * (self.kp_att * e_roll - self.kd_att * w[0])
        tau_y = INERTIA[1] * (self.kp_att * e_pitch - self.kd_att * w[1])
        tau_z = INERTIA[2] * (self.kp_yaw * e_yaw - self.kd_yaw * w[2])

        thrust = mass * az_total / max(0.3, math.cos(roll) * math.cos(pitch))
        thrust = min(max(thrust, 0.0), 4.0 * MAX_THRUST)

        u = tau_x / ARM
        v = tau_y / ARM
        w_ = tau_z / K_M
        t1 = (thrust - u - v - w_) / 4.0
        t2 = (thrust + u + v - w_) / 4.0
        t3 = (thrust + u + w_ - v) / 4.0
        t4 = (thrust + v + w_ - u) / 4.0
        thrusts = np.clip(np.array([t1, t2, t3, t4]), 0.0, MAX_THRUST)
        return thrusts, tau_z


class ZvController(PdController):
    """ZV（Zero Vibration）输入整形：两个 0.5 权重脉冲，间隔半个摆动周期。"""

    name = "zv"

    def __init__(self, settle_ratio=1.0, **kw):
        super().__init__(**kw)
        self.delay = 0.5 * SWING_PERIOD * settle_ratio
        self._hist = []

    def reset(self):
        self._hist = []

    def reference(self, sim, target):
        target = np.asarray(target, dtype=float)
        self._hist.append((sim.time, target.copy()))
        cutoff = sim.time - 10.0
        while len(self._hist) > 2 and self._hist[0][0] < cutoff:
            self._hist.pop(0)
        delayed = self._hist[0][1]
        for t, ref in self._hist:
            if t <= sim.time - self.delay + 1e-9:
                delayed = ref
            else:
                break
        return 0.5 * (target + delayed)


class SwingController(PdController):
    """摆角/摆速反馈：把机体往负载的水平投影处拉，抑制摆动。"""

    name = "swing"

    def __init__(self, k_swing=2.6, kd_swing=2.2, **kw):
        super().__init__(**kw)
        self.k_swing = k_swing
        self.kd_swing = kd_swing

    def accel(self, sim, ref):
        a_d = super().accel(sim, ref)
        dp = sim.payload_pos - sim.base_pos
        dv = sim.payload_vel - sim.base_vel
        a_d = a_d.copy()
        a_d[0] += self.k_swing * dp[0] + self.kd_swing * dv[0]
        a_d[1] += self.k_swing * dp[1] + self.kd_swing * dv[1]
        return a_d


CONTROLLERS = {
    "pd": PdController,
    "zv": ZvController,
    "swing": SwingController,
}
