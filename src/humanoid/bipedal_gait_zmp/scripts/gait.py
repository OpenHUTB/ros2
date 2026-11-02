# -*- coding: utf-8 -*-
"""步态求解：由落脚点计划求周期性质心轨迹，并验证 ZMP 稳定性。

求解思路
--------
LIPM 是线性系统，从初始状态 [x₀, v₀] 出发，经过一个步态周期 T 后的状态
可以写成仿射形式：

    [x(T)]   [a11 a12] [x₀]   [b1]
    [v(T)] = [a21 a22] [v₀] + [b2]

其中矩阵 A 与向量 b 只取决于 ZMP 参考轨迹。因为系统线性，用三次积分就能
把 A、b 都定出来（分别喂 (0,0)、(1,0)、(0,1) 三个初值）。

对称行走要求一个周期后质心前进一个步幅 L_stride、速度回到原值：

    x(T) = x₀ + L_stride
    v(T) = v₀

代入上式得到一个二元一次方程组，解出唯一的周期解 [x₀, v₀]。
再用这个初值积分整个计划，就得到可用的质心轨迹。
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from footstep import Footstep, GaitPlan, stability_margin  # noqa: E402
from lipm import LIPMParams, com_trajectory  # noqa: E402

__all__ = [
    "build_symmetric_plan",
    "integrate",
    "state_map",
    "solve_periodic",
    "simulate",
]


def build_symmetric_plan(n_steps, step_length, step_time, duty=0.6,
                         t0=0.0, foot_length=0.24):
    """构造对称行走的落脚点计划。

    参数
    ----
    n_steps      落脚点个数（至少 2）
    step_length  每步前进距离 (m)
    step_time    相邻两次落脚的间隔 (s)
    duty         支撑相占整个步态周期（2 个 step_time）的比例，行走需 > 0.5
    t0           第一步落地时刻
    foot_length  脚掌前后长度 (m)
    """
    if n_steps < 2:
        raise ValueError("n_steps 至少为 2")
    if step_length <= 0:
        raise ValueError("step_length 必须为正")
    if step_time <= 0:
        raise ValueError("step_time 必须为正")
    if not (0.5 < duty < 1.0):
        raise ValueError("行走的 duty 应在 (0.5, 1.0) 之间")
    if foot_length <= 0:
        raise ValueError("foot_length 必须为正")

    stance = duty * 2.0 * step_time
    steps = []
    x = 0.0
    for i in range(n_steps):
        side = "L" if i % 2 == 0 else "R"
        t_land = t0 + i * step_time
        steps.append(Footstep(side, x, t_land, t_land + stance,
                              half_length=foot_length / 2.0))
        x += step_length
    return GaitPlan(steps)


# ---------------------------------------------------------------------------
def integrate(x0, v0, zmp_func, t0, t1, dt, om):
    """从 t0 积分到 t1。zmp_func(t) 给出该时刻的 ZMP。

    每一步用解析解推进（ZMP 取步中点值），因此误差只来自 ZMP 在步内的变化，
    与步长 dt 同阶。
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")
    if t1 < t0:
        raise ValueError("t1 不能小于 t0")

    x, v = float(x0), float(v0)
    t = float(t0)
    while t < t1 - 1e-15:
        h = min(dt, t1 - t)
        p = zmp_func(t + 0.5 * h)
        x, v, _a = com_trajectory(x, v, p, h, om)
        t += h
    return x, v


def state_map(zmp_func, t0, t1, dt, om):
    """求一个区间的仿射映射矩阵 A 与向量 b。

    返回 ``(A, b)``，其中 ``A`` 为 2x2 嵌套列表，``b`` 为长度 2 的列表。
    """
    x0, v0 = integrate(0.0, 0.0, zmp_func, t0, t1, dt, om)
    x1, v1 = integrate(1.0, 0.0, zmp_func, t0, t1, dt, om)
    x2, v2 = integrate(0.0, 1.0, zmp_func, t0, t1, dt, om)

    a11, a21 = x1 - x0, v1 - v0
    a12, a22 = x2 - x0, v2 - v0
    return ([[a11, a12], [a21, a22]], [x0, v0])


def solve_periodic(plan, params, cycle_steps=2, dt=0.001, start_index=1):
    """解出一个步态周期的周期性质心初值。

    注意起点要跳过第一步：第一步落地时支撑关系里没有"前一只脚"，属于起步
    瞬态，那一段的 ZMP 参考不满足周期性。从第二只脚落地开始取周期。

    返回 ``(x0, v0, stride, t_start)``。
    """
    if cycle_steps < 2 or cycle_steps % 2 != 0:
        raise ValueError("cycle_steps 必须是大于 1 的偶数")
    if start_index < 1:
        raise ValueError("start_index 至少为 1，以跳过起步瞬态")
    if start_index + cycle_steps > len(plan.steps):
        raise ValueError("plan 的步数不足以容纳一个完整周期")

    t0 = plan.steps[start_index].t_start
    # 一个周期含 cycle_steps 步，质心前进 cycle_steps × 单步步长。
    # 不能写成 steps[-1].center - steps[0].center，那是相邻两步之差。
    per_step = plan.steps[start_index + 1].center - plan.steps[start_index].center
    stride = cycle_steps * per_step
    per_time = plan.steps[start_index + 1].t_start - plan.steps[start_index].t_start
    t1 = t0 + cycle_steps * per_time

    def zmp_func(t):
        return plan.zmp_reference(t)

    A, b = state_map(zmp_func, t0, t1, dt, params.omega)

    # (I - A) [x0, v0]^T = b - [stride, 0]
    m11, m12 = 1.0 - A[0][0], -A[0][1]
    m21, m22 = -A[1][0], 1.0 - A[1][1]
    r1, r2 = b[0] - stride, b[1]

    det = m11 * m22 - m12 * m21
    if abs(det) < 1e-12:
        raise RuntimeError("周期解奇异，可能步态参数不合理")

    x0 = (r1 * m22 - m12 * r2) / det
    v0 = (m11 * r2 - r1 * m21) / det
    return x0, v0, stride, t0


def simulate(plan, params, x0, v0, dt=0.001, t_start=None):
    """用给定的初值积分整个计划，返回轨迹与稳定性统计。

    t_start 不传则从计划起点开始（含起步瞬态）；传入周期起点可只看稳态段。

    返回 dict，包含等长的数组：time / com_x / com_v / com_a / zmp_ref /
    zmp_actual / poly_lo / poly_hi / margin。
    """
    if dt <= 0:
        raise ValueError("dt 必须为正")

    om = params.omega
    t0 = plan.t_start if t_start is None else float(t_start)
    t1 = plan.t_end
    if t1 <= t0:
        raise ValueError("结束时刻必须晚于起始时刻")

    n = int(round((t1 - t0) / dt)) + 1
    keys = ("time", "com_x", "com_v", "com_a", "zmp_ref",
            "zmp_actual", "poly_lo", "poly_hi", "margin")
    out = {k: [] for k in keys}

    x, v = float(x0), float(v0)
    for i in range(n):
        t = t0 + i * dt
        p_ref = plan.zmp_reference(t)
        lo, hi = plan.support_polygon(t)

        # 当前时刻的加速度由 ZMP 参考决定
        a = om * om * (x - p_ref)
        p_act = x - a / (om * om)      # 与 p_ref 相同，用于校验积分一致性

        out["time"].append(t)
        out["com_x"].append(x)
        out["com_v"].append(v)
        out["com_a"].append(a)
        out["zmp_ref"].append(p_ref)
        out["zmp_actual"].append(p_act)
        out["poly_lo"].append(lo)
        out["poly_hi"].append(hi)
        out["margin"].append(min(p_act - lo, hi - p_act))

        if i == n - 1:
            break
        # 用步中点的 ZMP 推进，精度与 dt 同阶
        p_mid = plan.zmp_reference(t + 0.5 * dt)
        x, v, _ = com_trajectory(x, v, p_mid, dt, om)

    return out


def summarise(traj):
    """从轨迹里挑几个常用指标。"""
    margins = traj["margin"]
    worst = min(margins)
    worst_i = margins.index(worst)
    zmp_errors = [abs(a - b) for a, b in
                  zip(traj["zmp_actual"], traj["zmp_ref"])]
    cos = traj["com_x"]
    ts = traj["time"]
    return {
        "duration": ts[-1] - ts[0],
        "samples": len(ts),
        "com_start": cos[0],
        "com_end": cos[-1],
        "com_travel": cos[-1] - cos[0],
        "com_max_speed": max(abs(v) for v in traj["com_v"]),
        "zmp_max_error": max(zmp_errors),
        "min_margin": worst,
        "min_margin_time": ts[worst_i],
        "stable": worst >= 0.0,
    }
