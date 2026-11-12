#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标准演示：估计相机相对 IMU 的时间偏移，并给出补偿前后的姿态误差。

    cd src/ground/camera_imu_time_calibration
    python3 -m camera_imu_time_calibration.main
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):  # 允许 python3 camera_imu_time_calibration/main.py 直接运行
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from camera_imu_time_calibration import core


def _plot_correlation(res, true_offset, out_dir, sim=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2),
                             gridspec_kw={"width_ratios": [2.0, 1.1]})
    tau_ms = res["grid"] * 1000.0
    ax = axes[0]
    ax.plot(tau_ms, res["rho"], color="#2b6cb0", linewidth=1.4,
            label="normalized correlation")
    ax.axvline(true_offset * 1000.0, color="#c53030", linestyle="--",
               linewidth=1.0, label=f"ground truth {true_offset*1000:.0f} ms")
    ax.plot([res["offset"] * 1000.0], [res["correlation"]], marker="o",
            markersize=5, color="#276749",
            label=f"estimate {res['offset']*1000:.2f} ms")
    core.style_axes(ax, "candidate offset $\\tau$ [ms]", "correlation $\\rho(\\tau)$",
                    "Coarse search ±250 ms")
    ax.legend(fontsize=7, loc="lower center")

    # 右图：峰附近 ±5 ms 的细搜曲线，说明亚毫秒分辨率来自哪里
    if sim is not None:
        fine = np.arange(-5.0, 5.001, 0.1) + res["offset"] * 1000.0
        rho_fine = [core.normalized_correlation(sim["t_cam"], sim["w_cam"],
                                                sim["t_imu"], sim["w_imu"],
                                                f / 1000.0) for f in fine]
        ax = axes[1]
        ax.plot(fine, rho_fine, color="#2b6cb0", linewidth=1.4)
        ax.axvline(true_offset * 1000.0, color="#c53030", linestyle="--",
                   linewidth=1.0)
        ax.plot([res["offset"] * 1000.0], [res["correlation"]], marker="o",
                markersize=5, color="#276749")
        core.style_axes(ax, "$\\tau$ [ms]", "", "Zoom ±5 ms")
        ax.set_ylim(min(rho_fine) - 2e-5, 1.0 + 5e-6)

    return core.savefig(fig, Path(out_dir) / "correlation_peak.png")


def _plot_timeseries(sim, d_hat, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = sim["t_true"]
    t_cam = sim["t_cam"]

    # 选斜率最大的 3 s 窗口，错位在快速转动时最容易看出来
    grad = np.abs(np.gradient(sim["w_true"][:, 2], t))
    center = float(t[int(np.argmax(grad))])
    window = (max(center - 1.5, t[0]), min(center + 1.5, t[-1]))
    sel = (t >= window[0]) & (t <= window[1])

    # 相机角速度重采样到真值网格：未补偿 / 已补偿
    def resample(shift):
        out = np.empty((len(t), 3))
        for a in range(3):
            out[:, a] = np.interp(t, t_cam - shift, sim["w_cam"][:, a])
        return out

    w_raw = resample(0.0)
    w_fix = resample(d_hat)

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 4.8), sharex=True)
    for ax, w, tag in ((axes[0], w_raw, "before: $\\tau$ = 0"),
                       (axes[1], w_fix, f"after: $\\tau$ = {d_hat*1000:.1f} ms")):
        ax.plot(t[sel], sim["w_true"][sel, 2], color="#2d3748",
                linewidth=1.4, label="ground truth $\\omega_z$")
        ax.plot(t[sel], w[sel, 2], color="#c53030", linewidth=0.9,
                linestyle=":", label="camera-derived $\\omega_z$")
        core.style_axes(ax, "time [s]", "$\\omega_z$ [rad/s]", tag)
    axes[0].legend(fontsize=7, loc="upper left")
    fig.suptitle("Camera angular rate vs IMU ground truth at the steepest segment",
                 fontsize=10)
    return core.savefig(fig, Path(out_dir) / "timeseries_alignment.png")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="相机-IMU 时间同步标定标准演示")
    parser.add_argument("--offset", type=float, default=50.0,
                        help="相机时间戳滞后真值的毫秒数（默认 50）")
    parser.add_argument("--cam-rate", type=float, default=30.0, help="相机帧率 Hz")
    parser.add_argument("--duration", type=float, default=20.0, help="时长 s")
    parser.add_argument("--seed", type=int, default=core.DEFAULT_SEED)
    parser.add_argument("--out-dir", default=None, help="图片输出目录")
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else core.default_out_dir()
    d_true = args.offset / 1000.0

    print("=" * 68)
    print("相机-IMU 时间同步标定：标准演示")
    print("=" * 68)
    print(f"时长 {args.duration:.0f} s   IMU 200 Hz   相机 {args.cam_rate:.0f} Hz   "
          f"真值偏移 {args.offset:.0f} ms   种子 {args.seed}")
    print(f"相机角速度噪声 0.010 rad/s   IMU 陀螺噪声 0.003 rad/s   零偏 0.005 rad/s")
    print(f"真值角速度峰值 |w| = {core.omega_peak():.3f} rad/s")
    print()

    sim = core.simulate(seed=args.seed, duration=args.duration,
                        cam_rate=args.cam_rate, offset=d_true)
    res = core.estimate_offset(sim["t_cam"], sim["w_cam"],
                               sim["t_imu"], sim["w_imu"])
    d_hat = res["offset"]

    print("-" * 68)
    print("时间偏移估计")
    print("-" * 68)
    print(f"  真值 d        = {d_true*1000:8.2f} ms")
    print(f"  估计 d_hat    = {d_hat*1000:8.2f} ms")
    print(f"  误差          = {(d_hat-d_true)*1000:8.2f} ms")
    print(f"  相关系数峰值  = {res['correlation']:.4f}")
    print(f"  尺度比估计    = {res['scale']:.4f} （真值 1.000，用于独立检查）")
    print(f"  参与对齐的帧数= {len(sim['t_cam'])} 帧 / {len(sim['t_imu'])} 个 IMU 样本")
    print()

    print("-" * 68)
    print("姿态误差（角速度积分 vs 真值姿态，全时长）")
    print("-" * 68)
    print(f"  {'处理方式':<14}{'τ [ms]':>10}{'最大误差 [°]':>16}{'RMS 误差 [°]':>16}")
    rows = [("未补偿", 0.0), ("估计值补偿", d_hat), ("理想补偿", d_true)]
    errs = {}
    for name, tau in rows:
        e = core.attitude_error(tau, sim)
        errs[name] = e
        print(f"  {name:<14}{tau*1000:>10.2f}{e['max_deg']:>16.2f}{e['rms_deg']:>16.2f}")
    e0, e1 = errs["未补偿"], errs["估计值补偿"]
    print()
    print(f"  RMS 误差由 {e0['rms_deg']:.2f}° 降到 {e1['rms_deg']:.2f}°，"
          f"降低 {100.0*(1.0-e1['rms_deg']/e0['rms_deg']):.0f}%")
    print(f"  30 Hz 相机本身的离散化底噪约 {core.attitude_error(d_true, sim)['rms_deg']:.2f}°，"
          f"估计值补偿已接近该底噪")
    print()

    print("-" * 68)
    print("输出图片")
    print("-" * 68)
    _plot_correlation(res, d_true, out_dir, sim=sim)
    _plot_timeseries(sim, d_hat, out_dir)
    print()
    print(f"图片目录：{out_dir}")
    print("下一步：python3 -m camera_imu_time_calibration.sweep_offset   # 偏移量扫描")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
