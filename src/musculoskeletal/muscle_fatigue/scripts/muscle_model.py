# -*- coding: utf-8 -*-
"""Hill 肌肉模型 + 疲劳因子。

在经典 Hill 模型基础上，给最大等长力乘一个随时间变化的疲劳因子 phi(t)：
肌肉持续激活时 phi 下降（能出的力衰减），休息时 phi 回升（恢复）。
本模块只用 Python 标准库，不需要 MuJoCo / ROS / matplotlib。

力-长度、力-速度、激活动力学的公式与 `muscle_joint_control` 版块保持一致：
总力 F = F_max * (a * f_l * f_v + f_p)，长度和速度都做归一化，
l_n = l / l_opt，v_n = v / v_max，v 为缩短速度（向心收缩为正）。
"""

from __future__ import annotations

import math

__all__ = [
    "MuscleParams",
    "FatigueParams",
    "activation_dynamics",
    "force_length_active",
    "force_length_passive",
    "force_velocity",
    "muscle_force",
    "fatigued_muscle_force",
    "fatigue_dynamics",
    "Muscle",
]


# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------

class MuscleParams(object):
    """一组肌肉参数，默认值取人体骨骼肌的常用量级。

    f_max      最大等长收缩力 (N)
    l_opt      最优肌纤维长度 (m)
    v_max      最大缩短速度 (m/s)
    tau_act    激活时间常数 (s)
    tau_deact  放松时间常数 (s)，应大于 tau_act
    width      主动力-长度曲线宽度
    """

    def __init__(self, f_max=100.0, l_opt=0.10, v_max=1.0,
                 tau_act=0.01, tau_deact=0.04, width=0.45):
        if f_max <= 0:
            raise ValueError("f_max 必须为正")
        if l_opt <= 0:
            raise ValueError("l_opt 必须为正")
        if v_max <= 0:
            raise ValueError("v_max 必须为正")
        if tau_act <= 0 or tau_deact <= 0:
            raise ValueError("时间常数必须为正")
        if tau_deact <= tau_act:
            raise ValueError("tau_deact 应大于 tau_act（放松慢于激活）")
        if width <= 0:
            raise ValueError("width 必须为正")
        self.f_max = float(f_max)
        self.l_opt = float(l_opt)
        self.v_max = float(v_max)
        self.tau_act = float(tau_act)
        self.tau_deact = float(tau_deact)
        self.width = float(width)

    def __repr__(self):
        return ("MuscleParams(f_max=%.1f, l_opt=%.3f, v_max=%.2f, "
                "tau_act=%.3f, tau_deact=%.3f, width=%.2f)"
                % (self.f_max, self.l_opt, self.v_max,
                   self.tau_act, self.tau_deact, self.width))


class FatigueParams(object):
    """疲劳因子 phi(t) 的一阶动力学参数。

    phi = 1 表示完全 fresh，phi 越小表示越疲劳。
    持续满激活时 phi 收敛到 phi_min（肌力下限，例如 0.55）；
    休息时 phi 以速率 k_rec 回升到 1。

    phi_min  持续激活下的疲劳下界（力可降到 phi_min 倍）
    k_fat   激活时疲劳累积速率 (1/s)
    k_rec   休息时恢复速率 (1/s)
    """

    def __init__(self, phi_min=0.55, k_fat=0.06, k_rec=0.03):
        if not (0.0 < phi_min < 1.0):
            raise ValueError("phi_min 必须介于 0 和 1 之间")
        if k_fat <= 0 or k_rec <= 0:
            raise ValueError("速率必须为正")
        self.phi_min = float(phi_min)
        self.k_fat = float(k_fat)
        self.k_rec = float(k_rec)

    def __repr__(self):
        return ("FatigueParams(phi_min=%.2f, k_fat=%.3f, k_rec=%.3f)"
                % (self.phi_min, self.k_fat, self.k_rec))


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ---------------------------------------------------------------------------
# 激活动力学
# ---------------------------------------------------------------------------

def activation_dynamics(u, a, dt, tau_act=0.01, tau_deact=0.04):
    """把神经指令 u 推进一个时间步，返回新的激活度。

    da/dt = (u - a) / tau，tau 在激活时用 tau_act，放松时用 tau_deact。
    u 与返回值都限幅在 [0, 1]。
    """
    u = _clip(float(u), 0.0, 1.0)
    a = _clip(float(a), 0.0, 1.0)
    dt = float(dt)
    if dt <= 0:
        raise ValueError("dt 必须为正")
    tau = tau_act if u > a else tau_deact
    a_new = a + (u - a) * dt / tau
    return _clip(a_new, 0.0, 1.0)


# ---------------------------------------------------------------------------
# 力-长度 / 力-速度 / 被动弹性
# ---------------------------------------------------------------------------

def force_length_active(l_n, width=0.45):
    """主动力-长度因子，最优长度处为 1。"""
    if width <= 0:
        raise ValueError("width 必须为正")
    return math.exp(-((l_n - 1.0) / width) ** 2)


def force_length_passive(l_n, k=5.0, l_max=1.6):
    """被动力-长度因子，l_n <= 1 时为 0，拉长后随结缔组织张紧上升。"""
    if l_n <= 1.0:
        return 0.0
    return (math.exp(k * (l_n - 1.0)) - 1.0) / (math.exp(k * (l_max - 1.0)) - 1.0)


def force_velocity(v_n):
    """力-速度因子，等长 (v_n=0) 为 1，向心收缩下降，离心收缩上升。

    v_n >= 1（缩短速度达到上限）后该因子会被夹取到 0：收缩元件只能拉不能推，
    Hill 曲线在 v_n > 1 处外推会给出负值，必须截断，否则总力变成推力。
    """
    if v_n >= 0.0:
        f = 0.3125 / (v_n + 0.25) - 0.25
        return max(0.0, f)
    x = min(1.0, -v_n)
    return 1.0 + 0.4 * x / (0.25 + x)


def muscle_force(a, l_n, v_n, params):
    """不计入疲劳的肌肉总力 (N)。"""
    if not isinstance(params, MuscleParams):
        raise TypeError("params 必须是 MuscleParams")
    a = _clip(float(a), 0.0, 1.0)
    fl = force_length_active(float(l_n), params.width)
    fv = force_velocity(float(v_n))
    fp = force_length_passive(float(l_n))
    return params.f_max * (a * fl * fv + fp)


def fatigued_muscle_force(a, l_n, v_n, phi, params):
    """计入疲劳因子的肌肉总力 (N)：最大力先乘疲劳因子 phi。"""
    phi = _clip(float(phi), 0.0, 1.0)
    return phi * muscle_force(a, l_n, v_n, params)


# ---------------------------------------------------------------------------
# 疲劳动力学
# ---------------------------------------------------------------------------

def fatigue_dynamics(a, phi, dt, fp):
    """把疲劳因子 phi 推进一个时间步，返回新的 phi。

    dphi/dt = k_rec * (1 - a) * (1 - phi) - k_fat * a * (phi - phi_min)

    休息 (a=0) 时 phi 回升到 1；持续激活 (a=1) 时 phi 收敛到 phi_min。
    """
    a = _clip(float(a), 0.0, 1.0)
    phi = _clip(float(phi), 0.0, 1.0)
    dt = float(dt)
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if not isinstance(fp, FatigueParams):
        raise TypeError("fp 必须是 FatigueParams")
    dphi = fp.k_rec * (1.0 - a) * (1.0 - phi) - fp.k_fat * a * (phi - fp.phi_min)
    return _clip(phi + dphi * dt, 0.0, 1.0)


# ---------------------------------------------------------------------------
# 状态化封装
# ---------------------------------------------------------------------------

class Muscle(object):
    """一块带疲劳状态的一自由度肌肉。

    用法::

        m = Muscle()
        for _ in range(steps):
            a = m.step(u=0.8, dt=0.01)      # 推进激活 + 疲劳
            F = m.force(l_n=1.0, v_n=0.0)   # 查询当前力
    """

    def __init__(self, params=None, fatigue=None, a0=0.0, phi0=1.0):
        self.params = params or MuscleParams()
        self.fatigue = fatigue or FatigueParams()
        self.a = _clip(float(a0), 0.0, 1.0)
        self.phi = _clip(float(phi0), 0.0, 1.0)

    def step(self, u, dt, l_n=1.0, v_n=0.0):
        """推进一个时间步，更新激活度与疲劳因子，返回当前力 (N)。"""
        self.a = activation_dynamics(u, self.a, dt,
                                     self.params.tau_act, self.params.tau_deact)
        self.phi = fatigue_dynamics(self.a, self.phi, dt, self.fatigue)
        return self.force(l_n, v_n)

    def force(self, l_n, v_n):
        return fatigued_muscle_force(self.a, l_n, v_n, self.phi, self.params)
