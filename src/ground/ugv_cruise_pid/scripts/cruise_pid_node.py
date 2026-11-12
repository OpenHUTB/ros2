#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UGV PID 巡航速度跟踪仿真（唯一实现）。

一维质点速度通道 + 带积分抗饱和与加速度限幅的 PID，在 0→5→8→3 m/s 阶跃
目标与 t=12–14 s 逆风阵风下闭环跟踪。输出指标、静态曲线 PNG 与逐帧动画
GIF，不依赖 ROS / Gazebo / CARLA。

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


class UGVPlant:
    """一维质点车辆：加速度 = 控制指令 / 质量 + 扰动，速度非负。"""

    def __init__(self, mass=1.0, dt=0.01):
        self.m = mass
        self.dt = dt
        self.v = 0.0
        self.x = 0.0

    def reset(self):
        self.v = 0.0
        self.x = 0.0

    def step(self, accel_cmd, disturbance=0.0):
        a = accel_cmd / self.m + disturbance
        self.v = max(0.0, self.v + a * self.dt)
        self.x += self.v * self.dt
        return self.v, self.x


class PIDCruiseController:
    """速度 PID：积分限幅抗饱和，加速度指令限幅。"""

    def __init__(self, dt=0.01, kp=3.0, ki=0.3, kd=0.5,
                 i_limit=3.0, a_limit=5.0):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self.a_limit = a_limit
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, v_target, v_meas):
        err = v_target - v_meas
        self.integral = float(np.clip(self.integral + err * self.dt,
                                      -self.i_limit, self.i_limit))
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        accel = self.kp * err + self.ki * self.integral + self.kd * deriv
        return float(np.clip(accel, -self.a_limit, self.a_limit))


def speed_profile(t):
    """阶跃目标速度：t<2 s 为 0，2–10 s 为 5 m/s，10–15 s 为 8 m/s，之后 3 m/s。"""
    if t < 2.0:
        return 0.0
    if t < 10.0:
        return 5.0
    if t < 15.0:
        return 8.0
    return 3.0


def wind_gust(t):
    """t=12–14 s 的逆风阵风，等效 -1.0 m/s² 加速度。"""
    return -1.0 if 12.0 < t < 14.0 else 0.0


def simulate(dt=0.01, t_end=20.0):
    """闭环仿真，返回 (times, v, v_target, x, accel_cmd)。"""
    plant = UGVPlant(dt=dt)
    ctrl = PIDCruiseController(dt=dt)
    times = np.arange(0.0, t_end, dt)
    vs = np.zeros_like(times)
    vts = np.zeros_like(times)
    xs = np.zeros_like(times)
    accels = np.zeros_like(times)
    for i, t in enumerate(times):
        vt = speed_profile(t)
        u = ctrl.update(vt, plant.v)
        v, x = plant.step(u, wind_gust(t))
        vs[i], vts[i], xs[i], accels[i] = v, vt, x, u
    return times, vs, vts, xs, accels


def rise_time(times, vs, target, low=0.1, high=0.9):
    """0→5 m/s 阶跃的 10%→90% 上升时间，窗口取该段阶跃内。"""
    mask = (times >= 2.0) & (times < 10.0)
    seg_t, seg_v = times[mask], vs[mask]
    if len(seg_t) == 0:
        return None
    lo = np.where(seg_v >= low * target)[0]
    hi = np.where(seg_v >= high * target)[0]
    if len(lo) == 0 or len(hi) == 0:
        return None
    return float(seg_t[hi[0]] - seg_t[lo[0]])


def evaluate(times, vs, vts, xs):
    """指标在窗口上分离测量：阶跃指标只取阵风前，阵风指标单独报告。"""
    # 8 m/s 阶跃稳态误差：11.0–11.9 s（阵风 12 s 起）。
    step_mask = (times >= 11.0) & (times < 11.9)
    steady_err = float(np.mean(np.abs(vts[step_mask] - vs[step_mask])))
    # 8 m/s 阶跃超调：10.0–12.0 s（阵风前）。
    pre_gust = vs[(times >= 10.0) & (times < 12.0)]
    overshoot = float(np.max(pre_gust) - 8.0)
    # 阵风峰值：12.0–14.0 s，相对 8 m/s 目标的最大偏差。
    gust_mask = (times >= 12.0) & (times < 14.0)
    gust_peak = float(np.max(np.abs(vts[gust_mask] - vs[gust_mask])))
    # 扰动后残差：14.5–14.9 s（阵风结束后、3 m/s 阶跃前）。
    res_mask = (times >= 14.5) & (times < 14.9)
    residual = float(np.max(np.abs(vts[res_mask] - vs[res_mask])))
    # 阶跃上升时间：2–10 s 段 0→5 m/s 的 10%→90%。
    t_rise = rise_time(times, vs, 5.0)
    return {
        "steady_err": steady_err,
        "overshoot": overshoot,
        "gust_peak": gust_peak,
        "residual": residual,
        "rise_time": t_rise,
        "final_speed": float(vs[-1]),
        "distance": float(xs[-1]),
    }


def plot_static(times, vs, vts, xs, accels, out_path):
    """静态两联图：速度跟踪 + 加速度指令。"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax1.plot(times, vts, "r--", lw=1.2, label="目标速度")
    ax1.plot(times, vs, "b-", lw=1.4, label="实际速度")
    ax1.axvspan(12.0, 14.0, color="orange", alpha=0.18, label="逆风阵风")
    ax1.set_ylabel("速度 (m/s)")
    ax1.set_title("地面车 PID 巡航速度跟踪")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.plot(times, accels, "g-", lw=1.2, label="加速度指令")
    ax2.set_ylabel("加速度 (m/s²)")
    ax2.set_xlabel("时间 (s)")
    ax2.legend(loc="lower right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gif(times, vs, vts, xs, accels, out_path, frames=60, fps=10):
    """逐帧动画：时间游标推进，速度曲线与加速度指令同步显示。"""
    step = max(1, len(times) // frames)
    idxs = list(range(0, len(times), step))[:frames]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    ax1.plot(times, vts, "r--", lw=1.0, label="目标速度")
    line_v, = ax1.plot([], [], "b-", lw=1.6, label="实际速度")
    dot_v, = ax1.plot([], [], "bo", ms=4)
    ax1.axvspan(12.0, 14.0, color="orange", alpha=0.18)
    ax1.set_xlim(0, times[-1])
    ax1.set_ylim(0, max(9.0, float(np.max(vs)) + 0.8))
    ax1.set_ylabel("速度 (m/s)")
    ax1.set_title("地面车 PID 巡航速度跟踪（动画）")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)
    line_a, = ax2.plot([], [], "g-", lw=1.4)
    dot_a, = ax2.plot([], [], "go", ms=4)
    ax2.set_xlim(0, times[-1])
    ax2.set_ylim(float(np.min(accels)) - 0.6, float(np.max(accels)) + 0.6)
    ax2.set_ylabel("加速度 (m/s²)")
    ax2.set_xlabel("时间 (s)")
    ax2.grid(alpha=0.3)
    cursor1 = ax1.axvline(0.0, color="gray", lw=0.8, alpha=0.6)
    cursor2 = ax2.axvline(0.0, color="gray", lw=0.8, alpha=0.6)
    txt = ax1.text(0.02, 0.92, "", transform=ax1.transAxes, fontsize=9)

    def update(i):
        k = idxs[i]
        line_v.set_data(times[:k], vs[:k])
        dot_v.set_data([times[k]], [vs[k]])
        line_a.set_data(times[:k], accels[:k])
        dot_a.set_data([times[k]], [accels[k]])
        cursor1.set_xdata([times[k], times[k]])
        cursor2.set_xdata([times[k], times[k]])
        txt.set_text("t = %.1f s   v = %.2f m/s   目标 %.1f" % (times[k], vs[k], vts[k])
                     + ("   （阵风）" if 12.0 < times[k] < 14.0 else ""))
        return line_v, dot_v, line_a, dot_a, cursor1, cursor2, txt

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=len(idxs), interval=1000 // fps, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="地面车 PID 巡航速度跟踪仿真")
    parser.add_argument("--out-dir", default="results", help="输出目录")
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--t-end", type=float, default=20.0)
    args = parser.parse_args(argv)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    times, vs, vts, xs, accels = simulate(dt=args.dt, t_end=args.t_end)
    m = evaluate(times, vs, vts, xs)

    print("=== 地面车 PID 巡航速度跟踪 ===")
    print("Python %s / numpy %s / matplotlib %s"
          % (sys.version.split()[0], np.__version__, matplotlib.__version__))
    print("末速 %.2f m/s（目标 %.1f）" % (m["final_speed"], vts[-1]))
    print("阶跃稳态误差(11.0–11.9 s) %.3f m/s" % m["steady_err"])
    print("阶跃超调(10–12 s) %.3f m/s" % m["overshoot"])
    print("阵风峰值(12–14 s) %.3f m/s" % m["gust_peak"])
    print("扰动后残差 %.3f m/s" % m["residual"])
    print("0→5 m/s 上升时间 %.2f s" % (m["rise_time"] or float("nan")))
    print("总行程 %.1f m" % m["distance"])

    png = os.path.join(out_dir, "cruise_response.png")
    gif = os.path.join(out_dir, "cruise_demo.gif")
    plot_static(times, vs, vts, xs, accels, png)
    plot_gif(times, vs, vts, xs, accels, gif)

    summary = {
        "module": "ugv_cruise_pid",
        "dt": args.dt,
        "t_end": args.t_end,
        "metrics": m,
    }
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("已保存：%s" % png)
    print("已保存：%s" % gif)
    print("已保存：%s" % os.path.join(out_dir, "summary.json"))
    return m


if __name__ == "__main__":
    main()
