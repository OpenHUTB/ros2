#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机 L1 自适应高度控制仿真（唯一实现）。

高度通道被控对象取竖直一维质点（质量 m = 1 kg），加速度由推力、重力与阵风
扰动叠加而成。控制器分三层：位置 PD 给出期望加速度 a_des；状态预测器与对象
同构并注入实测高度误差；自适应律用**速度预测误差**驱动扰动估计 sigma_hat，
估计量经一阶低通后进入控制律 u = m(a_des - lpf(sigma_hat))。高度指令在 t=1 s
由 0 阶跃到 2.0 m、t=10 s 再阶跃到 3.5 m；t=5–8 s 有下沉阵风，t>12 s 有持续
往复风扰。输出指标、静态曲线 PNG 与逐帧动画 GIF，不依赖 ROS / PX4 / Gazebo。

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

G = 9.81


class UAVAltitudePlant:
    """竖直一维质点：加速度 = 推力 / 质量 − 重力 + 风扰加速度。"""

    def __init__(self, mass=1.0, dt=0.001, floor=0.0):
        self.m = mass
        self.dt = dt
        self.floor = floor
        self.z = 0.0
        self.v = 0.0

    def reset(self):
        self.z = 0.0
        self.v = 0.0

    def step(self, thrust, wind=0.0):
        accel = thrust / self.m - G + wind
        self.v += accel * self.dt
        self.z += self.v * self.dt
        if self.floor is not None and self.z < self.floor:
            self.z = self.floor
            self.v = max(self.v, 0.0)
        return self.z, self.v


class L1AltitudeController:
    """L1 自适应高度控制器：PD 外环 + 状态预测器 + 速度误差驱动的扰动估计。"""

    def __init__(self, mass=1.0, dt=0.001, kp=4.0, kd=4.0, gamma=150.0,
                 k_pred=4.0, lpf_omega=5.0, sigma_limit=25.0, thrust_limit=20.0):
        self.m = mass
        self.dt = dt
        self.kp = kp
        self.kd = kd
        self.gamma = gamma
        self.k1 = k_pred
        self.k2 = k_pred
        self.lpf_alpha = lpf_omega
        self.sigma_limit = sigma_limit
        self.thrust_limit = thrust_limit
        self.reset()

    def reset(self):
        # 预测器与扰动估计都从「静止、已配平」出发：sigma_hat = -g 表示自重已被补偿
        self.z_hat = 0.0
        self.v_hat = 0.0
        self.sigma_hat = -G
        self.sigma_f = -G
        self.u_prev = self.m * G

    def update(self, z_cmd, z_meas, v_meas):
        ez = z_meas - self.z_hat
        # 状态预测器：与对象同构，用实测高度误差注入校正
        self.z_hat += self.dt * (self.v_hat + self.k1 * ez)
        self.v_hat += self.dt * (self.u_prev / self.m + self.sigma_hat + self.k2 * ez)
        # 自适应律：用速度预测误差驱动，再做投影限幅
        self.sigma_hat += self.gamma * (v_meas - self.v_hat) * self.dt
        self.sigma_hat = float(np.clip(self.sigma_hat, -self.sigma_limit, self.sigma_limit))
        # 低通滤波，保证进入执行器的补偿量平滑
        self.sigma_f += self.dt * self.lpf_alpha * (self.sigma_hat - self.sigma_f)

        a_des = self.kp * (z_cmd - z_meas) - self.kd * v_meas
        u = self.m * (a_des - self.sigma_f)
        u = float(np.clip(u, -self.thrust_limit, self.thrust_limit))
        self.u_prev = u
        return u


class PDAttitudeController:
    """对照基线：同样的 PD 与限幅，但把扰动估计固定为自重（无自适应）。"""

    def __init__(self, mass=1.0, kp=4.0, kd=4.0, thrust_limit=20.0):
        self.m = mass
        self.kp = kp
        self.kd = kd
        self.thrust_limit = thrust_limit

    def reset(self):
        pass

    def update(self, z_cmd, z_meas, v_meas):
        a_des = self.kp * (z_cmd - z_meas) - self.kd * v_meas
        u = self.m * (a_des + G)
        return float(np.clip(u, -self.thrust_limit, self.thrust_limit))


def altitude_profile(t):
    """高度指令：t<1 s 为 0，1–10 s 为 2.0 m，t≥10 s 为 3.5 m（两段阶跃）。"""
    t = np.asarray(t, dtype=float)
    return np.where(t < 1.0, 0.0, np.where(t < 10.0, 2.0, 3.5))


def wind(t):
    """t=5–8 s 下沉阵风 −2.0 m/s²，t>13 s 往复风扰 0.8 m/s²（0.6 Hz）。"""
    d = 0.0
    if 5.0 <= t < 8.0:
        d += -2.0
    if t >= 13.0:
        d += 0.8 * np.sin(2 * np.pi * 0.6 * (t - 13.0))
    return d


def simulate(controller="l1", dt=0.001, t_end=20.0):
    """闭环仿真，返回 (times, z, target, thrust, wind_arr, sigma_hat)。"""
    plant = UAVAltitudePlant(dt=dt)
    if controller == "l1":
        ctrl = L1AltitudeController(dt=dt)
    elif controller == "pd":
        ctrl = PDAttitudeController()
    else:
        raise ValueError("controller must be 'l1' or 'pd'")
    ctrl.reset()

    times = np.arange(0.0, t_end, dt)
    zs = np.zeros_like(times)
    targets = np.zeros_like(times)
    thrusts = np.zeros_like(times)
    winds = np.zeros_like(times)
    sigmas = np.zeros_like(times)

    for i, t in enumerate(times):
        tgt = float(altitude_profile(t))
        u = ctrl.update(tgt, plant.z, plant.v)
        z, _ = plant.step(u, wind(t))
        zs[i], targets[i], thrusts[i], winds[i] = z, tgt, u, wind(t)
        sigmas[i] = getattr(ctrl, "sigma_f", -G)
    return times, zs, targets, thrusts, winds, sigmas


def _first_cross(times, values, level, t0=0.0, t1=None):
    """窗口内首次满足 values >= level 的时刻。"""
    mask = times >= t0
    if t1 is not None:
        mask &= times < t1
    idx = np.where(mask & (values >= level))[0]
    return None if len(idx) == 0 else float(times[idx[0]])


def rise_time(times, zs, target, t0=1.0, t1=5.0, low=0.1, high=0.9):
    """首段阶跃（0 → 2.0 m）的 10% → 90% 上升时间，窗口止于阵风前。"""
    lo = _first_cross(times, zs, low * target, t0, t1)
    hi = _first_cross(times, zs, high * target, t0, t1)
    if lo is None or hi is None:
        return None
    return float(hi - lo)


def recovery_time(times, zs, target, t0=5.0, t1=8.0, band=0.02):
    """阵风窗口内偏差峰值出现后，回到 ±band 带所需的耗时。"""
    win = (times >= t0) & (times < t1)
    err = np.abs(zs - target)
    if not np.any(win):
        return None
    k = int(np.argmax(np.where(win, err, 0.0)))
    idx = np.where(win & (err <= band))[0]
    idx = idx[idx > k]
    if len(idx) == 0:
        return None
    return float(times[idx[0]] - times[k])


def settle_time(times, zs, targets, since=10.0, until=12.9, band=0.05):
    """第二段阶跃（2.0 → 3.5 m）进入 ±band 带并保持到风扰开始前的耗时。"""
    win = (times >= since) & (times < until)
    err = np.abs(targets - zs)
    idx = np.where(win & (err <= band))[0]
    if len(idx) == 0:
        return None
    for k in idx:
        tail = times[k:] < until
        if np.all(err[k:][tail] <= band):
            return float(times[k] - since)
    return None


def evaluate(times, zs, targets, thrusts):
    """指标按窗口分离：阶跃指标取扰动前，扰动指标单独报告。"""
    pre = (times >= 1.0) & (times < 5.0)
    overshoot = float(max(0.0, np.max(zs[pre]) - 2.0))
    steady_mask = (times >= 4.0) & (times < 4.9)
    steady_err = float(np.mean(np.abs(targets[steady_mask] - zs[steady_mask])))
    gust_mask = (times >= 5.0) & (times < 8.0)
    gust_peak = float(np.max(np.abs(targets[gust_mask] - zs[gust_mask])))
    res_mask = (times >= 7.9) & (times < 8.0)
    residual = float(np.max(np.abs(targets[res_mask] - zs[res_mask])))
    wave_mask = times >= 18.0
    wave_err = (float(np.mean(np.abs(targets[wave_mask] - zs[wave_mask])))
                if np.any(wave_mask) else None)
    return {
        "rise_time": rise_time(times, zs, 2.0),
        "overshoot": overshoot,
        "steady_err": steady_err,
        "gust_peak": gust_peak,
        "residual": residual,
        "recovery_time": recovery_time(times, zs, 2.0),
        "settle_time": settle_time(times, zs, targets),
        "wave_err": wave_err,
        "final_alt": float(zs[-1]),
        "max_thrust": float(np.max(np.abs(thrusts))),
    }


def plot_static(times, zs, targets, thrusts, winds, sigmas, out_path):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    ax1.plot(times, targets, "r--", lw=1.2, label="高度指令")
    ax1.plot(times, zs, "b-", lw=1.4, label="实际高度")
    ax1.axvspan(5.0, 8.0, color="orange", alpha=0.18, label="下沉阵风")
    ax1.axvspan(13.0, times[-1], color="skyblue", alpha=0.12, label="往复风扰")
    ax1.set_ylabel("高度 (m)")
    ax1.set_title("无人机 L1 自适应高度控制")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(times, thrusts, "g-", lw=1.0, label="推力")
    ax2.axhline(G, color="gray", ls=":", lw=1.0, label="悬停推力 9.81 N")
    ax2.set_ylabel("推力 (N)")
    ax2.legend(loc="lower right", fontsize=8)
    ax2.grid(alpha=0.3)

    ax3.plot(times, winds, color="orange", lw=1.0, label="真实风扰")
    ax3.plot(times, sigmas, "k-", lw=1.0, label="扰动估计（滤波后）")
    ax3.axhline(-G, color="gray", ls=":", lw=1.0, label="−g")
    ax3.set_ylabel("加速度当量 (m/s²)")
    ax3.set_xlabel("时间 (s)")
    ax3.legend(loc="lower right", fontsize=8)
    ax3.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def plot_gif(times, zs, targets, thrusts, winds, sigmas, out_path, frames=60, fps=10):
    """逐帧动画：左上高度曲线，左下推力，右侧竖直剖面里的机体位置。"""
    step = max(1, len(times) // frames)
    idxs = list(range(0, len(times), step))[:frames]
    fig = plt.figure(figsize=(9, 4.8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1.0], height_ratios=[1, 1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
    axp = fig.add_subplot(gs[:, 1])

    ax1.plot(times, targets, "r--", lw=1.0, label="高度指令")
    line_z, = ax1.plot([], [], "b-", lw=1.5, label="实际高度")
    dot_z, = ax1.plot([], [], "bo", ms=4)
    ax1.axvspan(5.0, 8.0, color="orange", alpha=0.18)
    ax1.axvspan(13.0, times[-1], color="skyblue", alpha=0.12)
    ax1.set_xlim(0, times[-1])
    ax1.set_ylim(-0.2, 4.2)
    ax1.set_ylabel("高度 (m)")
    ax1.set_title("无人机 L1 自适应定高（动画）")
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(alpha=0.3)

    line_u, = ax2.plot([], [], "g-", lw=1.2)
    ax2.set_xlim(0, times[-1])
    ax2.set_ylim(0, 22)
    ax2.set_ylabel("推力 (N)")
    ax2.set_xlabel("时间 (s)")
    ax2.grid(alpha=0.3)

    axp.set_xlim(-1.0, 1.0)
    axp.set_ylim(-0.3, 4.2)
    axp.axhline(0.0, color="tab:brown", lw=2.0)
    axp.axhline(2.0, color="r", ls="--", lw=1.0)
    axp.axhline(3.5, color="r", ls="--", lw=1.0)
    axp.set_yticks([0, 1, 2, 3, 4])
    axp.set_xticks([])
    axp.set_ylabel("高度 (m)")
    axp.set_title("竖直剖面", fontsize=9)
    body = plt.Rectangle((-0.18, 0.0), 0.36, 0.12, color="tab:blue")
    axp.add_patch(body)
    gust_arrow = axp.annotate("", xy=(0, 0.0), xytext=(0, 0.0),
                              arrowprops=dict(arrowstyle="->", color="orange", lw=2.0))
    txt1 = ax1.text(0.02, 0.88, "", transform=ax1.transAxes, fontsize=9)

    def update(i):
        k = idxs[i]
        line_z.set_data(times[:k], zs[:k])
        dot_z.set_data([times[k]], [zs[k]])
        line_u.set_data(times[:k], thrusts[:k])
        body.set_y(zs[k])
        if abs(winds[k]) > 0.01:
            gust_arrow.set_visible(True)
            base = zs[k]
            gust_arrow.xy = (0, base - 0.9 if winds[k] < 0 else base + 0.9)
            gust_arrow.set_position((0, base))
        else:
            gust_arrow.set_visible(False)
        txt1.set_text("t = %.1f s   高度 %.2f m   指令 %.2f m   风扰 %.2f m/s²   推力 %.1f N"
                      % (times[k], zs[k], targets[k], winds[k], thrusts[k]))
        return line_z, dot_z, line_u, body, gust_arrow, txt1

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=len(idxs), interval=1000 // fps, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="无人机 L1 自适应高度控制仿真")
    parser.add_argument("--out-dir", default="results", help="输出目录")
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--t-end", type=float, default=20.0)
    args = parser.parse_args(argv)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    times, zs, targets, thrusts, winds, sigmas = simulate("l1", dt=args.dt, t_end=args.t_end)
    m = evaluate(times, zs, targets, thrusts)
    _, zs_pd, _, thrusts_pd, _, _ = simulate("pd", dt=args.dt, t_end=args.t_end)
    m_pd = evaluate(times, zs_pd, targets, thrusts_pd)

    print("=== 无人机 L1 自适应高度控制 ===")
    print("Python %s / numpy %s / matplotlib %s"
          % (sys.version.split()[0], np.__version__, matplotlib.__version__))
    print("末高度 %.3f m（指令 3.5 m）" % m["final_alt"])
    print("0→2.0 m 上升时间 %.2f s" % (m["rise_time"] or float("nan")))
    print("首段超调 %.3f m" % m["overshoot"])
    print("首段稳态误差(4.0–4.9 s) %.3f m" % m["steady_err"])
    print("阵风峰值偏差(5–8 s) %.3f m" % m["gust_peak"])
    print("阵风后残差(7.9–8.0 s) %.3f m" % m["residual"])
    print("阵风恢复时间 %.2f s" % (m["recovery_time"] if m["recovery_time"] is not None
                                  else float("nan")))
    print("二段阶跃稳定时间 %.2f s" % (m["settle_time"] if m["settle_time"] is not None
                                    else float("nan")))
    print("往复风扰平均误差(18–20 s) %.3f m" % m["wave_err"])
    print("最大推力 %.1f N（限幅 20 N）" % m["max_thrust"])
    print("--- 无自适应 PD 基线 ---")
    print("阵风峰值偏差 %.3f m / 恢复时间 %.2f s / 风扰平均误差 %.3f m"
          % (m_pd["gust_peak"],
             m_pd["recovery_time"] if m_pd["recovery_time"] is not None else float("nan"),
             m_pd["wave_err"]))

    png = os.path.join(out_dir, "altitude_response.png")
    gif = os.path.join(out_dir, "altitude_demo.gif")
    plot_static(times, zs, targets, thrusts, winds, sigmas, png)
    plot_gif(times, zs, targets, thrusts, winds, sigmas, gif)

    summary = {
        "module": "uav_l1_altitude_stabilization",
        "dt": args.dt,
        "t_end": args.t_end,
        "metrics": m,
        "baseline_pd": m_pd,
    }
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print("已保存：%s" % png)
    print("已保存：%s" % gif)
    print("已保存：%s" % os.path.join(out_dir, "summary.json"))
    return m


if __name__ == "__main__":
    main()
