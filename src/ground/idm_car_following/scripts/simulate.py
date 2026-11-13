"""运行 IDM 跟车仿真并输出配图。"""

import os
import math
import idm as idm_mod
import simple_plot as sp


def lead_speed_single(t):
    """前车：15 m/s 巡航，t=5~7s 平滑减速到 8 m/s。"""
    if t < 5.0:
        return 15.0
    if t < 7.0:
        return 15.0 - 7.0 * (t - 5.0) / 2.0
    return 8.0


def lead_speed_pulse(t):
    """领航车：15 m/s 巡航，t=5~15s 一次平滑"减速-恢复"脉冲（余弦，谷值 10 m/s）。"""
    if t < 5.0 or t > 15.0:
        return 15.0
    return 15.0 - 5.0 * (0.5 - 0.5 * math.cos(2.0 * math.pi * (t - 5.0) / 10.0))


def run(outdir):
    os.makedirs(outdir, exist_ok=True)
    # 期望速度 v0=20 m/s（高于巡航速度 15，使跟随存在有限平衡间距）
    model = idm_mod.IDM(v0=20.0, T=1.5, a=2.0, b=3.0, s0=2.0)

    # 场景 A：单车跟驰（前车减速）
    ts, gaps, ve, vl = idm_mod.simulate_single(
        model, lead_speed_single, T=30.0, dt=0.01, ego_v0=12.0, gap0=30.0)
    fig1 = sp.Figure(width=720, height=420)
    fig1.plot(ts, gaps, color=sp.Figure.RED, label="GAP (M)")
    fig1.plot(ts, ve, color=sp.Figure.BLUE, label="EGO V (M/S)")
    fig1.plot(ts, vl, color=sp.Figure.GRAY, label="LEAD V (M/S)")
    fig1.xlabel = "t (s)"
    fig1.ylabel = "value"
    fig1.title = "IDM CAR-FOLLOWING: GAP AND SPEED"
    fig1.save(os.path.join(outdir, "idm_gap_speed.png"))

    # 场景 B：车队减速波向后传播
    n_follow = 5
    gap0 = model.equilibrium_gap(15.0)   # 以 15 m/s 的平衡间距初始化
    ts2, speeds = idm_mod.simulate_platoon(
        model, lead_speed_pulse, n_follow=n_follow, T=45.0, dt=0.01,
        v_init=15.0, gap_init=gap0)
    colors = [sp.Figure.RED, sp.Figure.BLUE, sp.Figure.GREEN, sp.Figure.ORANGE, sp.Figure.PURPLE]
    fig2 = sp.Figure(width=720, height=420)
    for i in range(1, n_follow + 1):
        fig2.plot(ts2, speeds[i], color=colors[(i - 1) % len(colors)], label="CAR%d" % i)
    fig2.xlabel = "t (s)"
    fig2.ylabel = "speed (m/s)"
    fig2.title = "BRAKE WAVE PROPAGATES BACKWARD"
    fig2.save(os.path.join(outdir, "idm_platoon.png"))

    # 统计：各车最终速度（应回到 15）、以及最小时距（防碰撞校验）
    final_speeds = [speeds[i][-1] for i in range(1, n_follow + 1)]
    return {
        "gap_end": gaps[-1],
        "ego_v_end": ve[-1],
        "lead_v_end": vl[-1],
        "platoon_final_speeds": final_speeds,
        "max_final_speed_spread": max(final_speeds) - min(final_speeds),
    }
