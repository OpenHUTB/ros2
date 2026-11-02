# -*- coding: utf-8 -*-
"""Hill 肌肉模型。

电机可以直接指定输出力矩，肌肉不行。肌肉只能被激活，神经给一个 0 到 1 的
激活度 a，肌肉自己根据当前长度和收缩速度决定出多大力。所以肌肉驱动关节的
控制量是激活度，不是力矩。

这里实现了四块：

    force_length_active    主动力跟长度的关系，最优长度处最大
    force_length_passive   被动力，拉长超过最优长度才有
    force_velocity         力跟速度的关系，离心收缩比等长收缩更有力
    activation_dynamics    激活度的一阶滞后，激活比放松快

总力是 F = F_max * (a * f_l * f_v + f_p)，长度和速度都做了归一化，
l_n = l / l_opt，v_n = v / v_max，v 是缩短速度，向心为正。
"""

from __future__ import annotations

import math

__all__ = [
    "MuscleParams",
    "activation_dynamics",
    "force_length_active",
    "force_length_passive",
    "force_velocity",
    "muscle_force",
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
    tau_deact  放松时间常数 (s)，应该比 tau_act 大
    width      主动力-长度曲线的宽度
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


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ---------------------------------------------------------------------------
# 1. 激活动力学
# ---------------------------------------------------------------------------

def activation_dynamics(u, a, dt, tau_act=0.01, tau_deact=0.04):
    """把神经指令 u 推进一个时间步，返回新的激活度。

    da/dt = (u - a) / tau，tau 在激活时用 tau_act，放松时用 tau_deact。
    u 和返回值都限幅在 [0, 1]。
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if tau_act <= 0 or tau_deact <= 0:
        raise ValueError("时间常数必须为正")

    u = _clip(float(u), 0.0, 1.0)
    a = _clip(float(a), 0.0, 1.0)
    tau = tau_act if u > a else tau_deact
    return _clip(a + dt * (u - a) / tau, 0.0, 1.0)


# ---------------------------------------------------------------------------
# 2. 主动力-长度关系
# ---------------------------------------------------------------------------

def force_length_active(l_norm, width=0.45):
    """归一化主动力-长度关系（高斯型）。

    ``l_norm = 1``（最优长度）时返回 1.0；偏离越多返回越小，且关于 1 对称。
    """
    if width <= 0:
        raise ValueError("width 必须为正")
    x = (float(l_norm) - 1.0) / width
    return math.exp(-x * x)


# ---------------------------------------------------------------------------
# 3. 被动力-长度关系
# ---------------------------------------------------------------------------

def force_length_passive(l_norm, k=5.0, l_max=1.6):
    """归一化被动力-长度关系（指数型）。

    l_norm <= 1 时返回 0，肌肉松弛不产生被动力。l_norm = l_max 时返回 1，
    再长就继续按指数涨。k 越大曲线越凸。
    """
    if k <= 0:
        raise ValueError("k 必须为正")
    if l_max <= 1.0:
        raise ValueError("l_max 必须大于 1")

    l_norm = float(l_norm)
    if l_norm <= 1.0:
        return 0.0
    num = math.exp(k * (l_norm - 1.0)) - 1.0
    den = math.exp(k * (l_max - 1.0)) - 1.0
    return num / den


# ---------------------------------------------------------------------------
# 4. 力-速度关系
# ---------------------------------------------------------------------------

#: Hill 方程中的形状常数（a/F0）
_HILL_A = 0.25
#: 离心收缩力的上限倍数
_ECC_MAX = 1.4


def force_velocity(v_norm):
    """归一化力-速度关系。

    ``v_norm = v / v_max``，其中 ``v`` 为**缩短速度**：

    * ``v_norm > 0`` 向心收缩（肌肉变短）→ 力小于 1，``v_norm = 1`` 时为 0
    * ``v_norm = 0`` 等长收缩 → 力为 1
    * ``v_norm < 0`` 离心收缩（肌肉被拉长）→ 力大于 1，渐近上限 1.4
    """
    v = float(v_norm)

    if v >= 0.0:
        # 由 (F + a)(v + b) = (F0 + a)b 归一化得到，a/F0 = b/vmax = 0.25
        denom = v + _HILL_A
        f = (_HILL_A * (1.0 + _HILL_A)) / denom - _HILL_A
        return max(0.0, f)

    # 离心段：单调上升到 _ECC_MAX
    x = -v
    if x > 1.0:
        x = 1.0
    return 1.0 + (_ECC_MAX - 1.0) * x / (_HILL_A + x)


# ---------------------------------------------------------------------------
# 5. 总肌肉力
# ---------------------------------------------------------------------------

def muscle_force(a, l_norm, v_norm, params=None):
    """按 Hill 模型算肌肉力 (N)。

    a 是激活度，l_norm 是 l / l_opt，v_norm 是缩短速度 v / v_max。
    params 不传就用默认参数。返回值是力，含被动力的部分。
    """
    if params is None:
        params = MuscleParams()

    a = _clip(float(a), 0.0, 1.0)
    f_l = force_length_active(l_norm, params.width)
    f_v = force_velocity(v_norm)
    f_p = force_length_passive(l_norm)

    active = a * f_l * f_v
    return params.f_max * (active + f_p)


# ---------------------------------------------------------------------------
# 便捷封装
# ---------------------------------------------------------------------------

class Muscle(object):
    """带内部激活状态的一块肌肉，便于在仿真循环里逐步推进。"""

    def __init__(self, params=None, activation=0.0):
        self.params = params if params is not None else MuscleParams()
        self.activation = _clip(float(activation), 0.0, 1.0)

    def update(self, u, dt):
        """按神经指令 u 推进一个时间步，返回当前肌肉力所需的数据。"""
        self.activation = activation_dynamics(
            u, self.activation, dt,
            self.params.tau_act, self.params.tau_deact)

    def force(self, l_norm, v_norm):
        """按当前激活度与给定的长度/速度计算肌肉力。"""
        return muscle_force(self.activation, l_norm, v_norm, self.params)

    def reset(self, activation=0.0):
        self.activation = _clip(float(activation), 0.0, 1.0)
