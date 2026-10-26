# -*- coding: utf-8 -*-
"""发散分量（DCM）反馈，用于稳住开环发散的双足步态。

为什么需要闭环
--------------
LIPM 的开环周期解确实存在，但它是个**不稳定平衡点**。仿真里任何微小误差
（积分误差、舍入误差）都会被放大，放大倍率约为 e^(ωT)。

这一点可以直接验证：把周期解作为初值积分，每过一个周期速度误差按固定倍数
增长。所以真实机器人不能只靠"算好的轨迹"，必须闭环。

发散分量
--------
定义 ξ = x + ẋ/ω。代入 LIPM 可以推出

    ξ̇ = ω (ξ − p)

这是**一阶不稳定系统**：ZMP 必须跑到 ξ 前面，才能把它拉回来。

稳走时质心平均速度 v_avg 恒定，可以取参考

    ξ_ref = p_ref + v_avg / ω

控制律（比例反馈 + 限幅）
------------------------
取 ZMP 指令为

    p = p_ref + K·(ξ − ξ_ref)

ξ 偏大（质心冲太快）时把 ZMP 往前压，反之往回拉。K = 0 就是纯开环跟踪
参考轨迹，会发散；K ≥ 1 才能稳住（见模块文档里的实测表）。最后把 p 限幅到
支撑多边形内——ZMP 不可能跑到脚外面去。

增益选择
--------
实测（步长 0.35 m、步时 0.5 s、duty 0.6、z_c 0.8 m）：

    K     行程        饱和比例   平均裕度
    0     发散        0%        0.136
    0.3   发散        82%       0.014
    1.0   3.93 m      20%       0.086
    2.0   4.04 m      47%       0.062

所以默认取 K = 1.0。
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from lipm import LIPMParams, com_trajectory, capture_point  # noqa: E402

__all__ = [
    "dcm",
    "dcm_reference",
    "zmp_feedback",
    "track",
    "summarise_track",
    "DEFAULT_GAIN",
]

#: 默认反馈增益
DEFAULT_GAIN = 1.0


def dcm(x, v, om):
    """发散分量 ξ = x + v / ω。"""
    if om <= 0:
        raise ValueError("omega 必须为正")
    return x + v / om


def dcm_reference(zmp_ref, v_avg, om):
    """稳走时的 DCM 参考值 ξ_ref = p_ref + v_avg / ω。"""
    if om <= 0:
        raise ValueError("omega 必须为正")
    return zmp_ref + v_avg / om


def zmp_feedback(p_ref, xi, xi_ref, gain=DEFAULT_GAIN):
    """比例反馈给出的 ZMP 指令（尚未限幅）。

        p = p_ref + gain · (ξ − ξ_ref)

    gain = 0 时退化为纯开环跟踪参考轨迹。
    """
    if gain < 0:
        raise ValueError("gain 不能为负")
    return p_ref + gain * (xi - xi_ref)


def _clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def track(plan, params, x0, v0, v_avg, dt=0.001, gain=DEFAULT_GAIN,
          saturate=True, t_start=None):
    """按 DCM 反馈走完整个计划，返回轨迹。

    与开环解的区别：每一步都用当前状态修正 ZMP，再限幅到支撑区间，因此误差
    不会累积放大。

    t_start 不传则从计划起点开始。注意初值必须与该时刻对应，否则一开始质心
    就在支撑区外，再怎么调 ZMP 也救不回来。

    返回 dict：time / com_x / com_v / com_a / zmp_ref / zmp_cmd（未限幅）/
    zmp_actual（限幅后）/ poly_lo / poly_hi / margin / dcm / dcm_ref /
    saturated
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")

    om = params.omega
    t0 = plan.t_start if t_start is None else float(t_start)
    t1 = plan.t_end
    if t1 <= t0:
        raise ValueError("结束时刻必须晚于起始时刻")
    n = int(round((t1 - t0) / dt)) + 1

    keys = ("time", "com_x", "com_v", "com_a", "zmp_ref", "zmp_cmd",
            "zmp_actual", "poly_lo", "poly_hi", "margin", "dcm", "dcm_ref",
            "saturated")
    out = {k: [] for k in keys}

    x, v = float(x0), float(v0)
    for i in range(n):
        t = t0 + i * dt
        p_ref = plan.zmp_reference(t)
        lo, hi = plan.support_polygon(t)

        xi = dcm(x, v, om)
        xi_ref = dcm_reference(p_ref, v_avg, om)
        p_cmd = zmp_feedback(p_ref, xi, xi_ref, gain)
        p = _clip(p_cmd, lo, hi) if saturate else p_cmd

        a = om * om * (x - p)

        out["time"].append(t)
        out["com_x"].append(x)
        out["com_v"].append(v)
        out["com_a"].append(a)
        out["zmp_ref"].append(p_ref)
        out["zmp_cmd"].append(p_cmd)
        out["zmp_actual"].append(p)
        out["poly_lo"].append(lo)
        out["poly_hi"].append(hi)
        out["margin"].append(min(p - lo, hi - p))
        out["dcm"].append(xi)
        out["dcm_ref"].append(xi_ref)
        out["saturated"].append(abs(p_cmd - p) > 1e-12)

        if i == n - 1:
            break
        x, v, _ = com_trajectory(x, v, p, dt, om)

    return out


def summarise_track(traj):
    """从闭环轨迹里挑指标。"""
    cos = traj["com_x"]
    ts = traj["time"]
    margins = traj["margin"]
    dcm_err = [abs(a - b) for a, b in zip(traj["dcm"], traj["dcm_ref"])]
    sat = sum(1 for s in traj["saturated"] if s)
    return {
        "duration": ts[-1] - ts[0],
        "samples": len(ts),
        "com_start": cos[0],
        "com_end": cos[-1],
        "com_travel": cos[-1] - cos[0],
        "com_max_speed": max(abs(v) for v in traj["com_v"]),
        "dcm_max_error": max(dcm_err),
        "dcm_mean_error": sum(dcm_err) / len(dcm_err),
        "min_margin": min(margins),
        "saturated_samples": sat,
        "saturated_ratio": sat / float(len(ts)),
        "stable": min(margins) >= -1e-9,
    }
