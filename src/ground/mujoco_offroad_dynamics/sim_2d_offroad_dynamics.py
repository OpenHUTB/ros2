#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_2d_offroad_dynamics.py —— 车辆动力学 · 越野地形 2D 轻量仿真（生成效果动图）
================================================================================
同一辆越野车在『干路→泥地→雪地→上坡』的越野路面上以 v0=30 m/s 紧急制动，
对比两种制动策略：
  - 无 ABS（红）：制动压力直接拉满 → 车轮锁死（滑移率 λ≈1）→ 轮胎摩擦掉到
    滑动摩擦 → 制动力弱、在低摩擦雪地上打滑滑行，制动距离长；
  - ABS（蓝）：实时把滑移率 λ 控制在峰值摩擦附近（λ≈0.2）→ 始终用最大
    可用摩擦 → 制动力强、稳定停车，制动距离大幅缩短。
下方速度曲线、上方面路摩擦剖面与两车位置、停车距离一起对比。

核心物理与 MuJoCo 渲染版共用 offroad_core.py；本文件可直接运行并生成动图。
运行:
  python3 sim_2d_offroad_dynamics.py [--frames 240] [--fps 30] [--out out.gif]
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle

from offroad_core import OffroadTerrain, run_brake


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=240)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", default="offroad_abs_demo.gif")
    ap.add_argument("--v0", type=float, default=30.0)
    args = ap.parse_args()

    v0 = args.v0
    terr = OffroadTerrain()
    veh = dict(m=1500, r=0.3, inertia=1.5, t_max=1.2e4)

    # 精确积分（core 用 dt=0.002）得到完整轨迹
    s_no, t_no, rec_no = run_brake(veh, terr, use_abs=False, v0=v0)
    s_abs, t_abs, rec_abs = run_brake(veh, terr, use_abs=True, v0=v0)
    T_total = max(t_no, t_abs)

    # 动画时间轴
    dt_anim = T_total / args.frames
    # 每条轨迹按动画帧索引取 (位置, 速度)
    def sample(rec, t_anim):
        ts = [r[0] for r in rec]
        idx = min(int(t_anim / ts[-1] * len(ts)), len(ts) - 1)
        return rec[idx]

    # ---------------- 绘图 ----------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.4, 9.2), gridspec_kw={'height_ratios': [1.6, 1.0]})
    fig.subplots_adjust(left=0.12, right=0.97, top=0.92, bottom=0.07, hspace=0.4)

    ax1.set_title("越野地形紧急制动 · ABS 对比（车辆动力学）", fontsize=12)
    ax1.set_xlim(0, terr.total); ax1.set_ylim(-1.2, 2.2)
    ax1.set_xlabel("路程 s (m)"); ax1.set_ylabel("")
    ax1.grid(alpha=0.3)

    # 路面摩擦剖面（分段色带）
    cmap = {'干路': '#7fb069', '泥地': '#b58863', '雪地': '#a8c8e8', '上坡': '#d0a44a'}
    for (a, b, mu, th) in terr.segments:
        lbl = terr.label_at((a + b) / 2)
        ax1.add_patch(Rectangle((a, -0.9), b - a, 0.6, fc=cmap[lbl], ec='0.5', lw=0.5, alpha=0.85))
        ax1.text((a + b) / 2, -0.6, f"{lbl}\nμ={mu:g}", ha='center', va='center', fontsize=8)

    # 两车
    car_no, = ax1.plot([], [], 'o', ms=12, mfc='#c0392b', mec='white', mew=1.5, label='无 ABS（轮锁死打滑）')
    car_ab, = ax1.plot([], [], 'o', ms=12, mfc='#2980b9', mec='white', mew=1.5, label='ABS（滑移率控制）')
    # 轨迹线
    trail_no, = ax1.plot([], [], '-', color='#c0392b', lw=1.8, alpha=0.7)
    trail_ab, = ax1.plot([], [], '-', color='#2980b9', lw=1.8, alpha=0.7)
    ax1.legend(loc='upper left', fontsize=8)
    # 停车距离文字（末帧）
    dist_no = ax1.text(0, 1.75, "", fontsize=9, color='#c0392b')
    dist_ab = ax1.text(0, 1.45, "", fontsize=9, color='#2980b9')

    # 速度子图
    ax2.set_title("车速 vs 时间", fontsize=10)
    ax2.set_xlim(0, T_total); ax2.set_ylim(0, v0 * 1.1)
    ax2.set_xlabel("t (s)"); ax2.set_ylabel("v (m/s)")
    ax2.grid(alpha=0.3)
    vno_l, = ax2.plot([], [], '-', color='#c0392b', lw=1.8, label='无 ABS v')
    vab_l, = ax2.plot([], [], '-', color='#2980b9', lw=1.8, label='ABS v')
    ax2.legend(loc='upper right', fontsize=8)

    info = fig.text(0.97, 0.005, '', ha='right', va='bottom', fontsize=9,
                    family='monospace', bbox=dict(fc='white', ec='0.6', alpha=0.92))
    imp = 100 * (1 - s_abs / s_no)

    def update(frame):
        t = frame * dt_anim
        rn = sample(rec_no, t); ra = sample(rec_abs, t)
        car_no.set_data([rn[1]], [1.0]); car_ab.set_data([ra[1]], [0.6])
        # 轨迹线：采样到当前时刻
        tn = [r[0] for r in rec_no]; ta = [r[0] for r in rec_abs]
        sn_hist = [r[1] for r in rec_no if r[0] <= t + 1e-9]
        sa_hist = [r[1] for r in rec_abs if r[0] <= t + 1e-9]
        trail_no.set_data(sn_hist, [1.0] * len(sn_hist))
        trail_ab.set_data(sa_hist, [0.6] * len(sa_hist))
        # 速度曲线
        vn_hist = [r[2] for r in rec_no if r[0] <= t + 1e-9]
        va_hist = [r[2] for r in rec_abs if r[0] <= t + 1e-9]
        tn_hist = [r[0] for r in rec_no if r[0] <= t + 1e-9]
        ta_hist = [r[0] for r in rec_abs if r[0] <= t + 1e-9]
        vno_l.set_data(tn_hist, vn_hist); vab_l.set_data(ta_hist, va_hist)

        done_no = rn[2] <= 0.05; done_ab = ra[2] <= 0.05
        dist_no.set_text(f"无ABS 停车 {s_no:.0f} m" if done_no else f"无ABS 位置 {rn[1]:.0f} m")
        dist_ab.set_text(f"ABS 停车 {s_abs:.0f} m" if done_ab else f"ABS 位置 {ra[1]:.0f} m")
        info.set_text(
            f"t = {t:.1f} s   v0 = {v0:.0f} m/s\n"
            f"路面: 干(μ0.9)→泥(μ0.5)→雪(μ0.3)→上坡\n"
            f"无ABS: 轮锁死 λ≈1.0 打滑  用时 {t_no:.1f}s  距离 {s_no:.0f} m\n"
            f"ABS  : 滑移率 λ≈0.2 峰值摩擦  用时 {t_abs:.1f}s  距离 {s_abs:.0f} m\n"
            f"ABS 缩短制动距离 ≈ {imp:.0f}%")
        return (car_no, car_ab, trail_no, trail_ab, vno_l, vab_l, dist_no, dist_ab, info)

    anim = FuncAnimation(fig, update, frames=args.frames, interval=1000 / args.fps, blit=False)
    anim.save(args.out, writer=PillowWriter(fps=args.fps), dpi=100)
    print(f"[OK] 动图已生成: {args.out}  ({args.frames} frames @ {args.fps}fps)")
    print(f"     无ABS 距离={s_no:.1f}m({t_no:.1f}s)  ABS 距离={s_abs:.1f}m({t_abs:.1f}s)  缩短={imp:.0f}%")


if __name__ == '__main__':
    main()
