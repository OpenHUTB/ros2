# -*- coding: utf-8 -*-
"""肌肉-肌腱串联弹性模型（muscle-tendon unit, MTU）。

肌肉不是直接挂在骨头上的：肌纤维（收缩成分）先接到肌腱（弹性成分），肌腱再
接到骨头上。两者串联，所以它们承受的力相同，长度相加：

    L_mtu = l_m + l_t

本模块在 Hill 肌纤维模型外面加了一根线性弹性肌腱，用来回答基础页里没有展开
的问题：

* 肌腱被拉长多少？能存多少弹性势能？
* 同样的激活度，串联肌腱以后传到关节上的力会小多少？
* 关节松开时，肌腱里的能量是怎么释放的？

约定
----
* 长度单位 m，力单位 N，时间 s，角度 rad。
* ``v_m`` 是**缩短速度**：肌纤维变短为正。这一点和 Hill 力-速度曲线的约定一致。
* 肌腱只有拉力，长度小于 slack length 时力为 0。
* 肌腱力-伸长关系取线性：F_t = k_t · max(0, l_t - l_slack)。

只用 Python 标准库，方便在没有 numpy 的环境里单独测试模型。
"""

from __future__ import annotations

import math

__all__ = [
    "TendonParams",
    "activation_step",
    "force_length_active",
    "force_length_passive",
    "force_velocity",
    "fiber_force",
    "tendon_force",
    "solve_fiber_length",
    "mtu_state",
]


class TendonParams(object):
    """一组肌肉-肌腱参数，默认值取人体上肢骨骼肌的常用量级。

    参数
    ----
    f_max : 最大等长收缩力 (N)
    l_opt : 肌纤维最优长度 (m)
    v_max : 最大缩短速度 (m/s)
    l_slack : 肌腱 slack length (m)，小于它时肌腱不受力
    tendon_strain : 肌腱在 f_max 下的应变，用来定肌腱刚度
    width : 主动力-长度曲线宽度
    passive_k : 被动力-长度曲线指数系数
    passive_l_max : 被动力归一化上限
    tau_act : 激活时间常数 (s)
    tau_deact : 放松时间常数 (s)，应大于 tau_act
    """

    def __init__(self, f_max=120.0, l_opt=0.10, v_max=1.2,
                 l_slack=0.05, tendon_strain=0.08, width=0.45,
                 passive_k=5.0, passive_l_max=1.6,
                 tau_act=0.01, tau_deact=0.04):
        if f_max <= 0:
            raise ValueError("f_max 必须为正")
        if l_opt <= 0:
            raise ValueError("l_opt 必须为正")
        if v_max <= 0:
            raise ValueError("v_max 必须为正")
        if l_slack <= 0:
            raise ValueError("l_slack 必须为正")
        if tendon_strain <= 0:
            raise ValueError("tendon_strain 必须为正")
        if width <= 0:
            raise ValueError("width 必须为正")
        if passive_k <= 0:
            raise ValueError("passive_k 必须为正")
        if passive_l_max <= 1.0:
            raise ValueError("passive_l_max 必须大于 1")
        if tau_act <= 0 or tau_deact <= 0:
            raise ValueError("时间常数必须为正")
        if tau_deact <= tau_act:
            raise ValueError("tau_deact 应大于 tau_act（放松慢于激活）")

        self.f_max = float(f_max)
        self.l_opt = float(l_opt)
        self.v_max = float(v_max)
        self.l_slack = float(l_slack)
        self.tendon_strain = float(tendon_strain)
        self.width = float(width)
        self.passive_k = float(passive_k)
        self.passive_l_max = float(passive_l_max)
        self.tau_act = float(tau_act)
        self.tau_deact = float(tau_deact)

    @property
    def tendon_stiffness(self):
        """名义肌腱刚度 k_t = F_max / (应变 · l_slack) (N/m)。"""
        if self.l_slack <= 0 or self.tendon_strain <= 0:
            raise ValueError("l_slack 和 tendon_strain 必须为正")
        return self.f_max / (self.tendon_strain * self.l_slack)

    def __repr__(self):
        return ("TendonParams(f_max=%.1f, l_opt=%.3f, l_slack=%.3f, "
                "strain=%.3f, k_t=%.0f)"
                % (self.f_max, self.l_opt, self.l_slack,
                   self.tendon_strain, self.tendon_stiffness))


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


# ---------------------------------------------------------------------------
# 1. 激活与肌纤维力
# ---------------------------------------------------------------------------

def activation_step(u, a, dt, tau_act=0.01, tau_deact=0.04):
    """把神经指令 u 推进一个时间步，返回新的激活度。

    da/dt = (u - a) / tau，激活时用 tau_act，放松时用 tau_deact。
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if tau_act <= 0 or tau_deact <= 0:
        raise ValueError("时间常数必须为正")

    u = _clip(float(u), 0.0, 1.0)
    a = _clip(float(a), 0.0, 1.0)
    tau = tau_act if u > a else tau_deact
    return _clip(a + dt * (u - a) / tau, 0.0, 1.0)


def force_length_active(l_norm, width=0.45):
    """归一化主动力-长度关系（高斯型），l_norm = 1 时为 1。"""
    if width <= 0:
        raise ValueError("width 必须为正")
    x = (float(l_norm) - 1.0) / width
    return math.exp(-x * x)


def force_length_passive(l_norm, k=5.0, l_max=1.6):
    """归一化被动力-长度关系（指数型），l_norm <= 1 时为 0。"""
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


#: Hill 方程形状常数 (a/F0 = b/vmax = 0.25)
_HILL_A = 0.25
#: 离心收缩力上限
_ECC_MAX = 1.4


def force_velocity(v_norm):
    """归一化力-速度关系，v_norm 为缩短速度 / v_max。

    * v_norm > 0 向心收缩，力小于 1，v_norm = 1 时为 0
    * v_norm = 0 等长收缩，力为 1
    * v_norm < 0 离心收缩，力大于 1，渐近上限 1.4
    """
    v = float(v_norm)
    if v >= 0.0:
        denom = v + _HILL_A
        f = (_HILL_A * (1.0 + _HILL_A)) / denom - _HILL_A
        return max(0.0, f)

    x = -v
    return 1.0 + (_ECC_MAX - 1.0) * x / (_HILL_A + x)


def fiber_force(activation, l_m, v_m, params=None):
    """按 Hill 模型算肌纤维力 (N)，含被动力。

    ``v_m`` 是缩短速度（变短为正）。
    """
    if params is None:
        params = TendonParams()
    a = _clip(float(activation), 0.0, 1.0)
    l_n = float(l_m) / params.l_opt
    v_n = float(v_m) / params.v_max
    active = a * force_length_active(l_n, params.width) * force_velocity(v_n)
    passive = force_length_passive(l_n, params.passive_k, params.passive_l_max)
    return params.f_max * (active + passive)


# ---------------------------------------------------------------------------
# 2. 肌腱
# ---------------------------------------------------------------------------

def tendon_force(l_t, params=None, k_t=None):
    """线性弹性肌腱力 (N)：F = k_t · max(0, l_t - l_slack)。"""
    if params is None:
        params = TendonParams()
    if k_t is None:
        k_t = params.tendon_stiffness
    if k_t <= 0:
        raise ValueError("k_t 必须为正")
    stretch = float(l_t) - params.l_slack
    return k_t * stretch if stretch > 0.0 else 0.0


def tendon_energy(force, k_t):
    """线性弹簧的弹性势能 E = F^2 / (2k)。比 0.5k·Δl² 更稳。"""
    if k_t <= 0:
        raise ValueError("k_t 必须为正")
    return float(force) * float(force) / (2.0 * k_t)


# ---------------------------------------------------------------------------
# 3. 串联平衡：给定 MTU 长度求肌纤维长度
# ---------------------------------------------------------------------------

def solve_fiber_length(mtu_length, activation, params=None, k_t=None,
                       velocity=0.0, max_stretch=0.06, tol=1e-12):
    """求满足 F_tendon(L - l_m) = F_fiber(l_m, v) 的肌纤维长度。

    肌腱和肌纤维串联、质量忽略，所以两者力相等。给定 MTU 总长度和激活度，
    用二分法解一维平衡。返回 ``(l_m, force)``。

    ``velocity`` 是当前肌纤维缩短速度，准静态实验传 0。
    """
    if params is None:
        params = TendonParams()
    if mtu_length <= 0:
        raise ValueError("mtu_length 必须为正")
    if k_t is None:
        k_t = params.tendon_stiffness
    if k_t <= 0:
        raise ValueError("k_t 必须为正")
    if max_stretch <= 0:
        raise ValueError("max_stretch 必须为正")

    L = float(mtu_length)
    hi = L - params.l_slack
    if hi <= 0.0:
        # 肌腱完全松弛，MTU 长度就是肌纤维长度，力为 0。
        return max(0.0, L), 0.0

    lo = max(1e-6, hi - float(max_stretch))

    def residual(l_m):
        f_t = tendon_force(L - l_m, params, k_t)
        f_m = fiber_force(activation, l_m, velocity, params)
        return f_t - f_m

    r_lo = residual(lo)
    r_hi = residual(hi)
    if r_lo <= 0.0:
        # 即使把肌腱拉到 max_stretch 也不够，说明参数超出模型适用范围。
        return lo, tendon_force(L - lo, params, k_t)
    if r_hi >= 0.0:
        return hi, 0.0

    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if residual(mid) > 0.0:
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break

    l_m = 0.5 * (lo + hi)
    return l_m, tendon_force(L - l_m, params, k_t)


def mtu_state(mtu_length, activation, params=None, k_t=None, velocity=0.0):
    """返回一条肌肉-肌腱单元的完整状态（dict）。

    字段：``l_m``、``l_t``、``force``、``fiber_force``、``tendon_force``、
    ``fiber_length_norm``、``tendon_strain``、``energy``。
    """
    if params is None:
        params = TendonParams()
    if k_t is None:
        k_t = params.tendon_stiffness

    l_m, force = solve_fiber_length(mtu_length, activation, params, k_t,
                                    velocity)
    l_t = float(mtu_length) - l_m
    f_m = fiber_force(activation, l_m, velocity, params)
    f_t = tendon_force(l_t, params, k_t)
    stretch = max(0.0, l_t - params.l_slack)
    return {
        "l_m": l_m,
        "l_t": l_t,
        "force": force,
        "fiber_force": f_m,
        "tendon_force": f_t,
        "fiber_length_norm": l_m / params.l_opt,
        "tendon_strain": stretch / params.l_slack if params.l_slack else 0.0,
        "energy": tendon_energy(force, k_t),
    }
