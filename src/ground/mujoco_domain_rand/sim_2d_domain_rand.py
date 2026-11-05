#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_2d_domain_rand.py —— 2D 轻量域随机化（Sim-to-Real）仿真，生成效果动图
================================================================================
展示『域随机化（Domain Randomization）』如何让仿真里标定的控制器对真实环境
的参数差异鲁棒（缩小 sim-to-real gap）。

场景：一辆地面小车沿参考速度剖面在折线路径上行驶，由『前馈控制器 u=g·v_ref』驱动。
  - 标称标定：用『仿真里以为的标称模型』求 g_nom = c0/k0 —— 只对仿真参数准；
  - 域随机化标定：在『真实环境可能的参数分布』上采样标定 g_dr —— 对整段分布平均准。
小车被部署到一组『真实参数』（电机弱、阻力大，与仿真不同）上，比较两控制器。

核心逻辑与 Gazebo/MuJoCo 渲染版共用 domain_rand_core.py；本文件可直接运行并生成动图。
运行:
  python3 sim_2d_domain_rand.py [--frames 200] [--fps 30] [--out out.gif]
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from domain_rand_core import (SecondOrderVehicle, FeedforwardController,
                              ref_speed, build_path, DomainRandomizer,
                              calibrate_nominal, calibrate_domain_randomized)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default="domain_rand_demo.gif")
    args = ap.parse_args()

    # 域随机化采样器：仿真标称 vs 真实分布（sim-to-real gap）
    sampler = DomainRandomizer(seed=0)
    g_nom = calibrate_nominal(sampler)          # 标称标定（仿真）
    g_dr = calibrate_domain_randomized(sampler)  # 域随机化标定（真实分布）

    # 一个代表性『真实部署参数』：电机弱、阻力大，明显偏离仿真标称
    real = (1.0, 0.75, 0.75)                    # (m, k, c)
    veh_n = SecondOrderVehicle(*real)           # 同一台真实车
    veh_d = SecondOrderVehicle(*real)
    ctrl_n = FeedforwardController(g_nom)
    ctrl_d = FeedforwardController(g_dr)

    # 参考路径与速度剖面
    way, interp, total = build_path()
    dt = 0.05
    T = args.frames * dt
    # 预积分参考弧长 s_ref(t) 并推进两台车
    sref_arr, ss_n, ss_d, v_n, v_d, vref = [], [], [], [], [], []
    s_ref = 0.0; s_n = 0.0; s_d = 0.0
    for k in range(args.frames):
        t = k * dt
        v_r = ref_speed(t)
        s_ref += v_r * dt
        u_n = ctrl_n.step(v_r, veh_n.v)
        v_nv, s_nv = veh_n.step(u_n, dt)
        u_d = ctrl_d.step(v_r, veh_d.v)
        v_dv, s_dv = veh_d.step(u_d, dt)
        sref_arr.append(s_ref); ss_n.append(s_nv); ss_d.append(s_dv)
        v_n.append(v_nv); v_d.append(v_dv); vref.append(v_r)

    sref_arr = np.array(sref_arr); ss_n = np.array(ss_n); ss_d = np.array(ss_d)
    v_n = np.array(v_n); v_d = np.array(v_d); vref = np.array(vref)
    rmse_n = float(np.sqrt(np.mean((v_n - vref) ** 2)))
    rmse_d = float(np.sqrt(np.mean((v_d - vref) ** 2)))

    # ---------------- 绘图 ----------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 9.0), gridspec_kw={'height_ratios': [1.6, 1.0]})
    fig.subplots_adjust(left=0.12, right=0.97, top=0.92, bottom=0.08, hspace=0.35)

    ax1.set_title("Domain Randomization · 前馈标定对比（Sim-to-Real）", fontsize=12)
    ax1.plot(way[:, 0], way[:, 1], '--', color='0.65', lw=1.2, label='参考路径')
    ax1.set_xlim(-4, 9); ax1.set_ylim(-2.5, 1.5)
    ax1.set_aspect('equal'); ax1.grid(alpha=0.3)
    ax1.set_xlabel('x (m)'); ax1.set_ylabel('y (m)')

    # 参考点、两车标记
    ref_pt, = ax1.plot([], [], 'g^', ms=11, mfc='#2e8b57', label='参考位置 (v_ref)')
    car_n, = ax1.plot([], [], 'o', ms=11, mfc='#c0392b', mec='white', mew=1.5, label='标称标定前馈')
    car_d, = ax1.plot([], [], 'o', ms=11, mfc='#2980b9', mec='white', mew=1.5, label='域随机化标定前馈')
    # 轨迹线
    trail_n, = ax1.plot([], [], '-', color='#c0392b', lw=1.6, alpha=0.7)
    trail_d, = ax1.plot([], [], '-', color='#2980b9', lw=1.6, alpha=0.7)
    ax1.legend(loc='upper right', fontsize=8)

    # 速度子图
    ax2.set_title("车速 vs 参考速度", fontsize=10)
    ax2.plot([0, T], [0, 0], 'k-', lw=0.5)
    ln_ref, = ax2.plot([], [], 'g-', lw=1.6, label='v_ref')
    ln_n, = ax2.plot([], [], '-', color='#c0392b', lw=1.6, label='标称前馈 v')
    ln_d, = ax2.plot([], [], '-', color='#2980b9', lw=1.6, label='DR 前馈 v')
    ax2.set_xlim(0, T); ax2.set_ylim(0, 1.6)
    ax2.set_xlabel('t (s)'); ax2.set_ylabel('v (m/s)')
    ax2.grid(alpha=0.3); ax2.legend(loc='upper right', fontsize=8)

    info = fig.text(0.97, 0.01, '', ha='right', va='bottom', fontsize=9,
                    family='monospace', bbox=dict(fc='white', ec='0.6', alpha=0.9))

    real_k = real[1]; real_c = real[2]
    title_txt = (f"真实部署参数: m={real[0]:.2f}  k={real_k:.2f}(电机弱×{real_k / sampler.k0:.2f})  "
                 f"c={real_c:.2f}(阻力大×{real_c / sampler.c0:.2f})\n"
                 f"g_nom(仿真标定)={g_nom:.3f}   g_dr(域随机化标定)={g_dr:.3f}")

    def update(frame):
        s_ref = sref_arr[frame]
        ref_xy_p = interp(s_ref)
        ref_pt.set_data([ref_xy_p[0]], [ref_xy_p[1]])
        pn = interp(ss_n[frame]); pd = interp(ss_d[frame])
        car_n.set_data([pn[0]], [pn[1]])
        car_d.set_data([pd[0]], [pd[1]])
        h = max(1, frame)
        trail_n.set_data([interp(s)[0] for s in ss_n[:h]],
                         [interp(s)[1] for s in ss_n[:h]])
        trail_d.set_data([interp(s)[0] for s in ss_d[:h]],
                         [interp(s)[1] for s in ss_d[:h]])
        tt = np.arange(frame + 1) * dt
        ln_ref.set_data(tt, vref[:frame + 1])
        ln_n.set_data(tt, v_n[:frame + 1])
        ln_d.set_data(tt, v_d[:frame + 1])
        info.set_text(
            f"t = {frame * dt:.1f} s\n{title_txt}\n"
            f"速度RMSE  标称 = {rmse_n:.2f} m/s   DR = {rmse_d:.2f} m/s\n"
            f"位置(弧长s) 标称 = {ss_n[frame]:.2f}   DR = {ss_d[frame]:.2f}   "
            f"参考 = {s_ref:.2f} m")
        return (ref_pt, car_n, car_d, trail_n, trail_d, ln_ref, ln_n, ln_d, info)

    anim = FuncAnimation(fig, update, frames=args.frames, interval=1000 / args.fps, blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=100)
    print(f"[OK] 动图已生成: {args.out}  ({args.frames} frames @ {args.fps}fps)")
    print(f"     标称 RMSE={rmse_n:.3f}   DR RMSE={rmse_d:.3f}   鲁棒性提升={100*(1-rmse_d/rmse_n):.0f}%")


if __name__ == '__main__':
    main()
