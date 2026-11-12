#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机航向角 PD 稳定控制仿真（唯一实现）。

偏航通道模型：转动惯量 Izz 的刚体受控制力矩与风扰力矩作用，航向角回绕到
[-π, π]。控制器为 PD（微分作用在角速率上），误差按最短弧回绕，力矩限幅。
航向指令在 t=7 s 由 +45° 阶跃到 -30°，t=5–6 s 有阵风力矩，t>10 s 有往复
风扰。输出指标、静态曲线 PNG 与逐帧动画 GIF，不依赖 ROS / Gazebo。

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


def wrap_pi(angle):
    """把角度回绕到 [-π, π]。"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class UAVYawPlant:
    """偏航单轴刚体：角加速度 = (控制力矩 + 扰动力矩) / Izz。"""

    def __init__(self, inertia=0.02, dt=0.01, rate_limit=None):
        self.Izz = inertia
        self.dt = dt
        self.rate_limit = rate_limit
        self.yaw = 0.0
        self.rate = 0.0

    def reset(self):
        self.yaw = 0.0
        self.rate = 0.0

    def step(self, torque, disturbance=0.0):
        accel = (torque + disturbance) / self.Izz
        self.rate += accel * self.dt
        if self.rate_limit is not None:
            self.rate = float(np.clip(self.rate, -self.rate_limit, self.rate_limit))
        self.yaw = wrap_pi(self.yaw + self.rate * self.dt)
        return self.yaw, self.rate


class PDYawController:
    """航向 PD：误差最短弧回绕，微分作用在角速率上，力矩限幅。"""

    def __init__(self, kp=1.2, kd=0.35, torque_limit=1.5):
        self.kp = kp
        self.kd = kd
        self.torque_limit = torque_limit

    def update(self, yaw_target, yaw_meas, yaw_rate_meas):
        err = wrap_pi(yaw_target - yaw_meas)
        u = self.kp * err - self.kd * yaw_rate_meas
        return float(np.clip(u, -self.torque_limit, self.torque_limit))


def yaw_profile(t):
    """航向指令：t<7 s 为 +45°，之后 -30°（两段阶跃转向）。

    标量与数组都可传入。
    """
    t = np.asarray(t, dtype=float)
    return np.where(t < 7.0, np.pi / 4, -np.pi / 6)


def wind_torque(t):
    """t=5–6 s 阵风力矩 0.06 N·m，t>10 s 往复风扰 0.02 N·m（0.8 Hz）。"""
    d = 0.0
    if 5.0 < t < 6.0:
        d += 0.06
    if t > 10.0:
        d += 0.02 * np.sin(2 * np.pi * 0.8 * (t - 10.0))
    return d


def simulate(dt=0.01, t_end=15.0):
    """闭环仿真，返回 (times, yaw, target, torque, rate)。"""
    plant = UAVYawPlant(dt=dt)
    ctrl = PDYawController()
    times = np.arange(0.0, t_end, dt)
    yaws = np.zeros_like(times)
    targets = np.zeros_like(times)
    torques = np.zeros_like(times)
    rates = np.zeros_like(times)
    for i, t in enumerate(times):
        tgt = yaw_profile(t)
        u = ctrl.update(tgt, plant.yaw, plant.rate)
        y, r = plant.step(u, wind_torque(t))
        yaws[i], targets[i], torques[i], rates[i] = y, tgt, u, r
    return times, yaws, targets, torques, rates


def rise_time(times, yaws, target, t0=0.0, t1=5.0, low=0.1, high=0.9):
    """首段阶跃（0→45°）的 10%→90% 上升时间，窗口止于阵风前。"""
    mask = (times >= t0) & (times < t1)
    seg_t, seg_y = times[mask], yaws[mask]
    lo = np.where(seg_y >= low * target)[0]
    hi = np.where(seg_y >= high * target)[0]
    if len(lo) == 0 or len(hi) == 0:
        return None
    return float(seg_t[hi[0]] - seg_t[lo[0]])


def recovery_time(times, yaws, target, since=6.0, band_deg=0.5):
    """阵风结束后首次进入 ±0.5° 带的耗时（相对 t=since）。"""
    mask = times >= since
    idx = np.where(mask & (np.abs(wrap_pi(target - yaws)) <= np.radians(band_deg)))[0]
    if len(idx) == 0:
        return None
    return float(times[idx[0]] - since)


def settle_time(times, yaws, target, since=7.0, until=10.0, band_deg=0.5):
    """第二段阶跃（+45°→-30°）进入 ±0.5° 带并在风扰开始前保持的耗时。"""
    err = np.degrees(np.abs(wrap_pi(yaw_profile(times) - yaws)))
    win = (times >= since) & (times < until)
    idx = np.where(win & (err <= band_deg))[0]
    if len(idx) == 0:
        return None
    for k in idx:
        if np.all(err[k:][times[k:] < until] <= band_deg):
            return float(times[k] - since)
    return None


def evaluate(times, yaws, targets, torques):
    """指标按窗口分离：阶跃指标取阵风前，阵风指标单独报告。"""
    step_mask = (times >= 4.0) & (times < 4.9)
    steady_err = float(np.degrees(np.mean(np.abs(targets[step_mask] - yaws[step_mask]))))
    pre_gust = yaws[times < 5.0]
    overshoot = float(np.degrees(np.max(pre_gust) - np.pi / 4))
    gust_mask = (times >= 5.0) & (times < 6.0)
    gust_peak = float(np.degrees(np.max(np.abs(wrap_pi(targets[gust_mask] - yaws[gust_mask])))))
    res_mask = (times >= 6.9) & (times < 7.0)
    residual = float(np.degrees(np.max(np.abs(wrap_pi(targets[res_mask] - yaws[res_mask])))))
    wave_mask = times >= 12.0
    wave_dev = float(np.degrees(np.max(np.abs(wrap_pi(targets[wave_mask] - yaws[wave_mask])))))
    return {
        "rise_time": rise_time(times, yaws, np.pi / 4),
        "overshoot": overshoot,
        "steady_err": steady_err,
        "gust_peak": gust_peak,
        "residual": residual,
        "recovery_time": recovery_time(times, yaws, np.pi / 4),
        "settle_time": settle_time(times, yaws, None),
        "wave_dev": wave_dev,
        "final_yaw": float(np.degrees(yaws[-1])),
        "max_torque": float(np.max(np.abs(torques))),
    }


def plot_static(times, yaws, targets, torques, out_path):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, np.degrees(targets), "r--", lw=1.2, label="航向指令")
    ax1.plot(times, np.degrees(yaws), "b-", lw=1.4, label="实际航向")
    ax1.axvspan(5.0, 6.0, color="orange", alpha=0.18, label="阵风力矩")
    ax1.axvspan(10.0, times[-1], color="skyblue", alpha=0.12, label="往复风扰")
    ax1.set_ylabel("航向角 (°)")
    ax1.set_title("无人机航向角 PD 稳定控制")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.plot(times, torques, "g-", lw=1.2, label="控制力矩")
    ax2.set_ylabel("力矩 (N·m)")
    ax2.set_xlabel("时间 (s)")
    ax2.legend(loc="lower right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gif(times, yaws, targets, torques, out_path, frames=60, fps=10):
    """逐帧动画：左上航向角曲线，右上力矩，下方俯视罗盘指针。"""
    step = max(1, len(times) // frames)
    idxs = list(range(0, len(times), step))[:frames]
    fig = plt.figure(figsize=(9, 4.8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1.0], height_ratios=[1, 1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
    axc = fig.add_subplot(gs[:, 1], projection="polar")

    ax1.plot(times, np.degrees(targets), "r--", lw=1.0, label="航向指令")
    line_y, = ax1.plot([], [], "b-", lw=1.5, label="实际航向")
    dot_y, = ax1.plot([], [], "bo", ms=4)
    ax1.axvspan(5.0, 6.0, color="orange", alpha=0.18)
    ax1.axvspan(10.0, times[-1], color="skyblue", alpha=0.12)
    ax1.set_xlim(0, times[-1])
    ax1.set_ylim(-60, 60)
    ax1.set_ylabel("航向角 (°)")
    ax1.set_title("无人机航向 PD（动画）")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    line_t, = ax2.plot([], [], "g-", lw=1.3)
    ax2.set_xlim(0, times[-1])
    ax2.set_ylim(-1.7, 1.7)
    ax2.set_ylabel("力矩 (N·m)")
    ax2.set_xlabel("时间 (s)")
    ax2.grid(alpha=0.3)
    txt1 = ax1.text(0.02, 0.88, "", transform=ax1.transAxes, fontsize=9)

    axc.set_theta_zero_location("N")
    axc.set_theta_direction(-1)
    axc.set_ylim(0, 1.0)
    axc.set_yticks([])
    axc.set_xticks(np.radians([0, 90, 180, 270]))
    axc.set_xticklabels(["0°", "90°", "180°", "270°"], fontsize=8)
    tgt_arrow = axc.annotate("", xy=(0, 0.92), xytext=(0, 0),
                             arrowprops=dict(arrowstyle="->", color="r", lw=1.4, ls="--"))
    act_arrow = axc.annotate("", xy=(0, 0.80), xytext=(0, 0),
                             arrowprops=dict(arrowstyle="->", color="tab:blue", lw=2.4))
    axc.set_title("俯视航向", fontsize=9)

    def update(i):
        k = idxs[i]
        line_y.set_data(times[:k], np.degrees(yaws[:k]))
        dot_y.set_data([times[k]], [np.degrees(yaws[k])])
        line_t.set_data(times[:k], torques[:k])
        tgt_arrow.xy = (targets[k], 0.92)
        act_arrow.xy = (yaws[k], 0.80)
        txt1.set_text("t = %.1f s   航向 %.1f°   指令 %.1f°   力矩 %.2f N·m"
                      % (times[k], np.degrees(yaws[k]), np.degrees(targets[k]), torques[k]))
        return line_y, dot_y, line_t, tgt_arrow, act_arrow, txt1

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=len(idxs), interval=1000 // fps, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="无人机航向角 PD 稳定控制仿真")
    parser.add_argument("--out-dir", default="results", help="输出目录")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--t-end", type=float, default=15.0)
    args = parser.parse_args(argv)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    times, yaws, targets, torques, rates = simulate(dt=args.dt, t_end=args.t_end)
    m = evaluate(times, yaws, targets, torques)

    print("=== 无人机航向角 PD 稳定控制 ===")
    print("Python %s / numpy %s / matplotlib %s"
          % (sys.version.split()[0], np.__version__, matplotlib.__version__))
    print("末航向 %.1f°（指令 -30.0°）" % m["final_yaw"])
    print("0→45° 上升时间 %.2f s" % (m["rise_time"] or float("nan")))
    print("首段超调 %.3f°" % m["overshoot"])
    print("首段稳态误差(4.0–4.9 s) %.3f°" % m["steady_err"])
    print("阵风峰值(5–6 s) %.3f°" % m["gust_peak"])
    print("阵风后残差 %.3f°" % m["residual"])
    print("阵风恢复时间 %.2f s" % (m["recovery_time"] if m["recovery_time"] is not None
                                  else float("nan")))
    print("二段阶跃稳定时间 %.2f s" % (m["settle_time"] if m["settle_time"] is not None
                                    else float("nan")))
    print("往复风扰最大偏差(12–15 s) %.3f°" % m["wave_dev"])
    print("最大力矩 %.2f N·m" % m["max_torque"])

    png = os.path.join(out_dir, "yaw_response.png")
    gif = os.path.join(out_dir, "yaw_demo.gif")
    plot_static(times, yaws, targets, torques, png)
    plot_gif(times, yaws, targets, torques, gif)

    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump({"module": "uav_yaw_stabilize", "dt": args.dt, "t_end": args.t_end,
                   "metrics": m}, f, ensure_ascii=False, indent=2)

    print("已保存：%s" % png)
    print("已保存：%s" % gif)
    print("已保存：%s" % os.path.join(out_dir, "summary.json"))
    return m


if __name__ == "__main__":
    main()
