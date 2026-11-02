# -*- coding: utf-8 -*-
"""关节刚度分析。

关节力矩 tau = r * (F_flex - F_ext)。稳定的关节，tau 和 theta 是反号的，
所以刚度定义成

    k = -d(tau)/d(theta)       k > 0 表示稳定

展开一下能看出刚度从哪来：

    d(l_flex)/d(theta) = -r,  d(l_ext)/d(theta) = +r
    k = r^2 * ( dF_flex/dl + dF_ext/dl )

也就是刚度正比于两条肌肉力-长度曲线斜率之和。

共激活的影响取决于肌肉在曲线的哪一段：上升段上激活越高刚度越大，最优长度
处没影响，下降段上反而越小甚至变负。下降段能不能稳住看被动力够不够抵消：

    a * d(f_l)/dl + d(f_p)/dl > 0
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from joint_model import JointParams, MuscleJoint  # noqa: E402
from muscle_model import (  # noqa: E402
    force_length_active,
    force_length_passive,
    muscle_force,
)

__all__ = [
    "static_torque",
    "joint_stiffness",
    "operating_segment",
    "slope_balance",
    "stiffness_sweep",
    "disturbance_response",
    "energy_cost",
    "tradeoff_analysis",
]

#: 判定"接近最优长度"的相对容差
_OPT_TOL = 0.005


# ---------------------------------------------------------------------------
# 1. 静态力矩与刚度
# ---------------------------------------------------------------------------

def static_torque(params, theta, a_flex, a_ext):
    """等长（v=0）条件下的关节力矩 (N·m)。

    正值使关节正转。两侧同样激活、且肌肉对称时，θ=0 处力矩为 0。
    """
    r = params.moment_arm
    mp = params.muscle_params()
    l_flex = params.rest_length - r * theta
    l_ext = params.rest_length + r * theta
    f_flex = muscle_force(a_flex, l_flex / params.l_opt, 0.0, mp)
    f_ext = muscle_force(a_ext, l_ext / params.l_opt, 0.0, mp)
    return r * (f_flex - f_ext)


def joint_stiffness(params, theta=0.0, a_flex=0.2, a_ext=0.2, dtheta=1e-6):
    """关节刚度 k = −∂τ/∂θ，数值中心差分。

    k > 0 表示存在恢复力矩（稳定）；k < 0 表示偏转会被放大（失稳）。
    """
    if dtheta <= 0:
        raise ValueError("dtheta 必须为正")
    tp = static_torque(params, theta + dtheta, a_flex, a_ext)
    tm = static_torque(params, theta - dtheta, a_flex, a_ext)
    return -(tp - tm) / (2.0 * dtheta)


# ---------------------------------------------------------------------------
# 2. 工作点判定与机理分解
# ---------------------------------------------------------------------------

def operating_segment(params):
    """判断肌肉在 θ=0 处工作在力-长度曲线的哪一段。

    返回 ``"上升段"`` / ``"最优长度"`` / ``"下降段"`` / ``"过度拉长"``。
    """
    ratio = params.rest_length / params.l_opt
    if ratio > 1.6:
        return "过度拉长"
    if abs(ratio - 1.0) <= _OPT_TOL:
        return "最优长度"
    return "上升段" if ratio < 1.0 else "下降段"


def slope_balance(params, a=0.5):
    """把刚度来源拆成"主动项"与"被动项"，用于解释机理。

    返回 ``dict``，字段：

    * ``l_norm``      归一化长度
    * ``f_l`` / ``f_p`` 主动、被动力
    * ``active``      ``a · ∂f_l/∂l_n``（下降段为负）
    * ``passive``     ``∂f_p/∂l_n``（l_n ≤ 1 时为 0）
    * ``total``       两者之和，> 0 才稳定
    * ``stable``      布尔
    """
    l_n = params.rest_length / params.l_opt
    mp = params.muscle_params()
    d = 1e-6

    dfl = (force_length_active(l_n + d, mp.width)
           - force_length_active(l_n - d, mp.width)) / (2 * d)
    dfp = (force_length_passive(l_n + d) - force_length_passive(l_n - d)) / (2 * d)
    active = a * dfl
    total = active + dfp

    return {
        "l_norm": l_n,
        "f_l": force_length_active(l_n, mp.width),
        "f_p": force_length_passive(l_n),
        "active": active,
        "passive": dfp,
        "total": total,
        "stable": total > 0.0,
    }


# ---------------------------------------------------------------------------
# 3. 共激活扫描
# ---------------------------------------------------------------------------

def stiffness_sweep(params, co_levels, theta=0.0):
    """扫描共激活水平，返回 ``[(co, k), ...]``。

    共激活指两侧肌肉同时维持的激活度（屈肌 = 伸肌 = co）。
    """
    if not co_levels:
        raise ValueError("co_levels 不能为空")
    out = []
    for co in co_levels:
        if co < 0.0 or co > 1.0:
            raise ValueError("共激活水平必须在 [0, 1] 内")
        out.append((float(co), joint_stiffness(params, theta, co, co)))
    return out


# ---------------------------------------------------------------------------
# 4. 扰动响应
# ---------------------------------------------------------------------------

def disturbance_response(params, co=0.2, tau_ext=0.05, duration=1.5, dt=0.001,
                         use_controller=False, kp=9.0, kd=0.9):
    """施加恒定外力矩，测量稳态偏转与等效刚度。

    参数
    ----
    co : 共激活水平（两侧基础激活度）
    tau_ext : 外加力矩 (N·m)，正值使关节正转
    use_controller : True 时叠加 PD 位置控制（目标 0），False 时只有共激活

    返回
    ----
    dict : 含 ``theta_ss``（稳态偏转）、``k_equivalent``（等效刚度）、
           ``max_deflection``、``history``
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if duration <= 0:
        raise ValueError("duration 必须为正")

    j = MuscleJoint(params)
    steps = int(round(duration / dt))
    h = j.history

    for i in range(steps):
        t = i * dt
        if use_controller:
            tau_cmd = kp * (0.0 - j.theta) - kd * j.theta_dot
            tau_max = params.f_max * params.moment_arm
            ratio = tau_cmd / tau_max
            if ratio >= 0.0:
                u_flex = co + (1.0 - co) * min(1.0, ratio)
                u_ext = co
            else:
                u_flex = co
                u_ext = co + (1.0 - co) * min(1.0, -ratio)
        else:
            u_flex = u_ext = co

        j.step(u_flex, u_ext, dt, tau_ext=tau_ext)
        j.record(t)

    theta = h["theta"]
    n = max(1, len(theta) // 10)
    theta_ss = sum(theta[-n:]) / len(theta[-n:])
    max_def = max(abs(x) for x in theta)

    k_eq = abs(tau_ext / theta_ss) if abs(theta_ss) > 1e-12 else float("inf")

    return {
        "theta_ss": theta_ss,
        "max_deflection": max_def,
        "k_equivalent": k_eq,
        "history": h,
        "co": co,
        "tau_ext": tau_ext,
    }


# ---------------------------------------------------------------------------
# 5. 能耗
# ---------------------------------------------------------------------------

def energy_cost(co, duration, activation_heat=1.0):
    """共激活维持一段时间的热代价（无量纲代理量）。

    肌肉维持激活本身就要消耗代谢能量（与是否做功无关），这里用
    ``∫ a dt`` 作为代理：两条肌肉各贡献 ``co``，持续 ``duration``。
    """
    if co < 0.0 or co > 1.0:
        raise ValueError("共激活水平必须在 [0, 1] 内")
    if duration < 0:
        raise ValueError("duration 不能为负")
    return 2.0 * co * duration * activation_heat


# ---------------------------------------------------------------------------
# 6. 刚度-精度权衡
# ---------------------------------------------------------------------------

def tradeoff_analysis(params, co_levels, tau_ext=0.05, duration=1.2, dt=0.001,
                      target=0.3, kp=9.0, kd=0.9):
    """在同一组共激活水平上同时评估刚度、跟踪精度与能耗代价。

    返回 ``list[dict]``，每项含 ``co`` / ``k`` / ``ss_error`` /
    ``rise_time`` / ``energy``。
    """
    from joint_model import simulate_step_response  # 局部导入，避免循环引用

    rows = []
    for co in co_levels:
        k = joint_stiffness(params, 0.0, co, co)
        res = simulate_step_response(target, duration=duration, dt=dt,
                                     kp=kp, kd=kd, co_activation=co,
                                     joint_params=params)
        m = res["metrics"]
        rows.append({
            "co": float(co),
            "k": k,
            "ss_error": m["steady_state_error"],
            "rise_time": m["rise_time"],
            "overshoot": m["overshoot_percent"],
            "energy": energy_cost(co, duration),
        })
    return rows
