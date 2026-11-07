# -*- coding: utf-8 -*-
"""滑模控制（SMC）：二阶系统的鲁棒轨迹跟踪（只用 Python 标准库）。

被控对象（含未知有界扰动）：
    m * x_ddot + c * x_dot + k * x = u + d(t)

控制目标：让 x(t) 跟踪参考轨迹 r(t)。

SMC 思路：
    - 定义滑模面 s = e_dot + lambda * e,  e = x - r
    - 等效控制抵消标称动力学，切换项 -K*sat(s/phi) 抑制扰动 d(t)
    - 只要 K 大于扰动幅值上界，系统就进入滑动模态，跟踪误差被压到边界层量级

作为对照，同时给出一阶 PD（含标称前馈但无扰动抑制）的结果。
"""

from __future__ import annotations

import math

DT = 0.005
T_END = 10.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def reference(t):
    """平滑参考：0 -> 1 的平滑阶跃（logistic），5s 后叠加正弦摆动。

    用平滑过渡而非硬阶跃，避免参考导数出现尖峰。
    """
    step = 1.0 / (1.0 + math.exp(-(t - 1.0) / 0.15))
    if t < 5.0:
        return step
    return step + 0.4 * math.sin(1.2 * (t - 5.0))


def disturbance(t):
    """未知有界扰动：均值 2.0、峰值约 3.5 的持续干扰。"""
    return 1.5 * math.sin(1.7 * t) + 2.0


def sat(z, phi):
    """边界层符号函数，phi 越大抖振越小但稳态残差越大。"""
    return math.tanh(z / phi)


def simulate_smc(m=1.0, c=0.6, k=2.0, lam=3.0, K=5.0, phi=0.08):
    """跑一次 SMC 仿真，返回 (ts, xs, rs)。"""
    x = 0.0
    xdot = 0.0
    r_prev = 0.0
    rdot_prev = 0.0
    e_prev = 0.0
    ts, xs, rs = [], [], []
    n = int(round(T_END / DT))
    for i in range(n + 1):
        t = i * DT
        r = reference(t)
        rdot = clamp((r - r_prev) / DT, -20.0, 20.0)
        rddot = clamp((rdot - rdot_prev) / DT, -40.0, 40.0)
        e = x - r
        edot = clamp((e - e_prev) / DT, -40.0, 40.0)
        s = edot + lam * e
        u = m * (rddot - lam * edot) + c * xdot + k * x - K * sat(s, phi)
        d = disturbance(t)
        xddot = (u + d - c * xdot - k * x) / m
        ts.append(t)
        xs.append(x)
        rs.append(r)
        xdot += xddot * DT
        x += xdot * DT
        r_prev = r
        rdot_prev = rdot
        e_prev = e
    return ts, xs, rs


def simulate_pd(m=1.0, c=0.6, k=2.0, Kp=8.0, Kd=4.0):
    """对照：标称前馈 + PD，无主动扰动抑制。"""
    x = 0.0
    xdot = 0.0
    r_prev = 0.0
    rdot_prev = 0.0
    e_prev = 0.0
    ts, xs, rs = [], [], []
    n = int(round(T_END / DT))
    for i in range(n + 1):
        t = i * DT
        r = reference(t)
        rdot = clamp((r - r_prev) / DT, -20.0, 20.0)
        rddot = clamp((rdot - rdot_prev) / DT, -40.0, 40.0)
        e = x - r
        edot = clamp((e - e_prev) / DT, -40.0, 40.0)
        # 标称前馈用当前状态 (x, xdot) 抵消，反馈项取负号以保证稳定
        u = m * rddot + c * xdot + k * x - Kp * e - Kd * edot
        d = disturbance(t)
        xddot = (u + d - c * xdot - k * x) / m
        ts.append(t)
        xs.append(x)
        rs.append(r)
        xdot += xddot * DT
        x += xdot * DT
        r_prev = r
        rdot_prev = rdot
        e_prev = e
    return ts, xs, rs


def mean_abs_error(xs, rs):
    errs = [abs(x - r) for x, r in zip(xs, rs)]
    return sum(errs) / len(errs)


def steady_error(xs, rs, t0=6.0):
    """t>=t0 之后的平均绝对误差（看稳态）。"""
    pairs = [(x, r) for t, x, r in zip(range(len(xs)), xs, rs)
             if t * DT >= t0]
    if not pairs:
        return 0.0
    return sum(abs(x - r) for x, r in pairs) / len(pairs)


if __name__ == "__main__":
    ts, xs_smc, rs = simulate_smc()
    _, xs_pd, _ = simulate_pd()
    print("SMC 全程平均误差 : %.4f" % mean_abs_error(xs_smc, rs))
    print("PD  全程平均误差 : %.4f" % mean_abs_error(xs_pd, rs))
    print("SMC 稳态误差     : %.4f" % steady_error(xs_smc, rs))
    print("PD  稳态误差     : %.4f" % steady_error(xs_pd, rs))
