#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROV 深度保持 PID 仿真（唯一实现）。

竖直通道模型：质量 m 的刚体在水下受推力、浮力扰动与波浪扰动的合力，
深度非负。控制器为带积分抗饱和的 PID，深度指令在 t=1 s 由 0 阶跃到 2 m；
t=5–8 s 出现负载浮力上浮扰动（-1.5 m/s²），t>12 s 叠加波浪往复扰动。
输出指标、静态曲线 PNG 与逐帧动画 GIF，不依赖 ROS / Gazebo。

用法::

    python3 main.py --out-dir results
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class ROVDepthPlant:
    """ROV 竖直方向一维动力学（质量 + 附加质量合并为等效质量）。"""

    def __init__(self, mass=5.0, dt=0.01):
        self.m = mass
        self.dt = dt
        self.depth = 0.0
        self.vel = 0.0

    def reset(self):
        self.depth = 0.0
        self.vel = 0.0

    def step(self, thrust, disturbance=0.0):
        accel = thrust / self.m + disturbance
        self.vel += accel * self.dt
        self.depth = max(0.0, self.depth + self.vel * self.dt)
        return self.depth, self.vel


class PIDDepthController:
    """深度 PID：积分限幅抗饱和，输出推力限幅。"""

    def __init__(self, dt=0.01, kp=16.0, ki=0.4, kd=20.0,
                 i_limit=10.0, u_limit=40.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.u_limit = u_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, depth_target, depth_meas, vel_meas):
        # 微分项作用在测量上（-kd·v），避免指令阶跃时的微分冲击。
        err = depth_target - depth_meas
        self.integral = float(np.clip(self.integral + err * self.dt,
                                      -self.i_limit, self.i_limit))
        self.prev_err = err
        u = self.kp * err + self.ki * self.integral - self.kd * vel_meas
        return float(np.clip(u, -self.u_limit, self.u_limit))


def depth_profile(t):
    """深度指令：t<1 s 保持 0（水面等待），之后阶跃到 2.0 m 并保持。"""
    return 0.0 if t < 1.0 else 2.0


def buoyancy_disturbance(t):
    """t=5–8 s 负载上浮（-1.5 m/s²），t>12 s 波浪往复（0.5 m/s²，0.3 Hz）。"""
    d = 0.0
    if 5.0 < t < 8.0:
        d -= 1.5
    if t > 12.0:
        d += 0.5 * np.sin(2 * np.pi * 0.3 * (t - 12.0))
    return d


def simulate(dt=0.01, t_end=20.0, target_hold=True):
    """闭环仿真，返回 (times, depth, target, thrust, vel)。"""
    plant = ROVDepthPlant(dt=dt)
    ctrl = PIDDepthController(dt=dt)
    times = np.arange(0.0, t_end, dt)
    depths = np.zeros_like(times)
    targets = np.zeros_like(times)
    thrusts = np.zeros_like(times)
    vels = np.zeros_like(times)
    for i, t in enumerate(times):
        tgt = depth_profile(t) if target_hold else 2.0
        u = ctrl.update(tgt, plant.depth, plant.vel)
        d, v = plant.step(u, buoyancy_disturbance(t))
        depths[i], targets[i], thrusts[i], vels[i] = d, tgt, u, v
    return times, depths, targets, thrusts, vels


def rise_time(times, depths, target, low=0.1, high=0.9):
    """1 s 之后的 10%→90% 上升时间（相对 2 m 阶跃幅值）。"""
    mask = (times >= 1.0) & (times < 5.0)
    seg_t, seg_d = times[mask], depths[mask]
    lo = np.where(seg_d >= low * target)[0]
    hi = np.where(seg_d >= high * target)[0]
    if len(lo) == 0 or len(hi) == 0:
        return None
    return float(seg_t[hi[0]] - seg_t[lo[0]])


def recovery_time(times, depths, target, since=8.0, band=0.02):
    """扰动结束后首次回到 ±2% 带内的耗时（相对 t=since）。"""
    mask = times >= since
    idx = np.where(mask & (np.abs(target - depths) <= band * target))[0]
    if len(idx) == 0:
        return None
    return float(times[idx[0]] - since)


def evaluate(times, depths, targets, thrusts):
    """指标按窗口分离：阶跃指标避开扰动窗口，扰动指标单独报告。"""
    target = 2.0
    # 阶跃稳态误差：4.0–4.9 s（扰动 5 s 起）。
    step_mask = (times >= 4.0) & (times < 4.9)
    steady_err = float(np.mean(np.abs(targets[step_mask] - depths[step_mask])))
    # 阶跃超调：1–5 s 的最大超调量。
    pre_dist = depths[(times >= 1.0) & (times < 5.0)]
    overshoot = float(np.max(pre_dist) - target)
    # 上浮扰动峰值偏差：5–8 s。
    dist_mask = (times >= 5.0) & (times < 8.0)
    dist_peak = float(np.max(np.abs(targets[dist_mask] - depths[dist_mask])))
    # 扰动恢复时间：8 s 起回到 ±2%（0.04 m）。
    rec = recovery_time(times, depths, target, since=8.0)
    # 波浪段残余：18–20 s 平均绝对误差。
    wave_mask = (times >= 18.0)
    wave_err = float(np.mean(np.abs(targets[wave_mask] - depths[wave_mask])))
    return {
        "rise_time": rise_time(times, depths, target),
        "overshoot": overshoot,
        "steady_err": steady_err,
        "dist_peak": dist_peak,
        "recovery_time": rec,
        "wave_err": wave_err,
        "final_depth": float(depths[-1]),
        "max_thrust": float(np.max(np.abs(thrusts))),
    }


def plot_static(times, depths, targets, thrusts, out_path):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, targets, "r--", lw=1.2, label="深度指令")
    ax1.plot(times, depths, "b-", lw=1.4, label="实际深度")
    ax1.axvspan(5.0, 8.0, color="orange", alpha=0.18, label="负载上浮扰动")
    ax1.axvspan(12.0, times[-1], color="skyblue", alpha=0.12, label="波浪扰动")
    ax1.set_ylabel("深度 (m)")
    ax1.set_title("ROV 深度保持 PID")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.plot(times, thrusts, "g-", lw=1.2, label="推力指令")
    ax2.set_ylabel("推力 (N)")
    ax2.set_xlabel("时间 (s)")
    ax2.legend(loc="lower right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gif(times, depths, targets, thrusts, out_path, frames=60, fps=10):
    """逐帧动画：左为深度-推力曲线，右为竖直剖面里的 ROV 位置。"""
    step = max(1, len(times) // frames)
    idxs = list(range(0, len(times), step))[:frames]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4.6),
                                   gridspec_kw={"width_ratios": [2.0, 1.0]})
    ax1.plot(times, targets, "r--", lw=1.0, label="深度指令")
    line_d, = ax1.plot([], [], "b-", lw=1.5, label="实际深度")
    dot_d, = ax1.plot([], [], "bo", ms=4)
    ax1.axvspan(5.0, 8.0, color="orange", alpha=0.18)
    ax1.axvspan(12.0, times[-1], color="skyblue", alpha=0.12)
    ax1.set_xlim(0, times[-1])
    ax1.set_ylim(-0.2, 3.0)
    ax1.set_xlabel("时间 (s)")
    ax1.set_ylabel("深度 (m)")
    ax1.set_title("ROV 深度保持（动画）")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    txt1 = ax1.text(0.02, 0.90, "", transform=ax1.transAxes, fontsize=9)

    ax2.set_xlim(-1.0, 1.0)
    ax2.set_ylim(3.0, -0.3)
    ax2.set_xticks([])
    ax2.set_ylabel("深度 (m)")
    ax2.set_title("竖直剖面")
    ax2.axhline(0.0, color="deepskyblue", lw=2)
    ax2.text(0.0, -0.18, "水面", ha="center", fontsize=8, color="deepskyblue")
    ax2.axhline(2.0, color="r", ls="--", lw=1)
    rov, = ax2.plot([], [], "s", ms=16, color="tab:blue")
    thrust_arrow = ax2.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->", color="g", lw=1.6))
    ax2.grid(alpha=0.2, axis="y")

    def update(i):
        k = idxs[i]
        line_d.set_data(times[:k], depths[:k])
        dot_d.set_data([times[k]], [depths[k]])
        rov.set_data([0.0], [depths[k]])
        # 推力箭头：长度随推力归一化，向上为正（上浮）
        ln = float(np.clip(-thrusts[k] / 20.0, -0.6, 0.6))
        thrust_arrow.xy = (0.0, depths[k] - ln * 0.6)
        thrust_arrow.set_position((0.0, depths[k]))
        txt1.set_text("t = %.1f s   深度 %.2f m   指令 %.1f m   推力 %.1f N"
                      % (times[k], depths[k], targets[k], thrusts[k]))
        return line_d, dot_d, rov, thrust_arrow, txt1

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=len(idxs), interval=1000 // fps, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="ROV 深度保持 PID 仿真")
    parser.add_argument("--out-dir", default="results", help="输出目录")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--t-end", type=float, default=20.0)
    args = parser.parse_args(argv)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    times, depths, targets, thrusts, vels = simulate(dt=args.dt, t_end=args.t_end)
    m = evaluate(times, depths, targets, thrusts)

    print("=== ROV 深度保持 PID ===")
    print("Python %s / numpy %s / matplotlib %s"
          % (sys.version.split()[0], np.__version__, matplotlib.__version__))
    print("末深度 %.3f m（指令 2.0）" % m["final_depth"])
    print("0→2 m 上升时间 %.2f s" % (m["rise_time"] or float("nan")))
    print("超调 %.3f m" % m["overshoot"])
    print("阶跃稳态误差(4.0–4.9 s) %.3f m" % m["steady_err"])
    print("上浮扰动峰值(5–8 s) %.3f m" % m["dist_peak"])
    print("扰动恢复时间 %.2f s" % (m["recovery_time"] if m["recovery_time"] is not None
                                  else float("nan")))
    print("波浪段平均误差(18–20 s) %.3f m" % m["wave_err"])
    print("最大推力 %.1f N" % m["max_thrust"])

    png = os.path.join(out_dir, "depth_response.png")
    gif = os.path.join(out_dir, "depth_demo.gif")
    plot_static(times, depths, targets, thrusts, png)
    plot_gif(times, depths, targets, thrusts, gif)

    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"module": "rov_depth_hold", "dt": args.dt, "t_end": args.t_end,
                   "metrics": m}, f, ensure_ascii=False, indent=2)

    print("已保存：%s" % png)
    print("已保存：%s" % gif)
    print("已保存：%s" % os.path.join(out_dir, "summary.json"))
    return m


if __name__ == "__main__":
    main()
