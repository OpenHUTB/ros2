#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""鲁棒性实验：相机噪声、IMU 零偏、相机尺度误差、运动激励幅度。

重点回答两个问题：估计器对哪些误差源免疫，对哪些敏感。

    cd src/ground/camera_imu_time_calibration
    python3 -m camera_imu_time_calibration.sweep_noise --trials 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from camera_imu_time_calibration import core

D_TRUE_MS = 50.0
CAM_NOISES = (0.0, 0.005, 0.010, 0.020, 0.050, 0.100, 0.200)
IMU_BIASES = (0.0, 0.010, 0.050, 0.100, 0.200, 0.500)
CAM_GAINS = (0.90, 0.95, 1.00, 1.05, 1.10, 1.20, 1.30)
EXCITATIONS = (1.00, 0.50, 0.25, 0.10, 0.05, 0.02)


def _sweep(param, values, trials):
    """对某个仿真参数做扫描，返回 (偏移误差 ms, 尺度比, 相关系数) 的均值。"""
    err, scale, rho = [], [], []
    for v in values:
        e, s, r = [], [], []
        for k in range(trials):
            _, res = core.trial(seed=k, offset=D_TRUE_MS / 1000.0, **{param: v})
            e.append((res["offset"] - D_TRUE_MS / 1000.0) * 1000.0)
            s.append(res["scale"])
            r.append(res["correlation"])
        err.append((float(np.mean(e)), float(np.std(e))))
        scale.append(float(np.mean(s)))
        rho.append(float(np.mean(r)))
    return np.asarray(err), np.asarray(scale), np.asarray(rho)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="相机-IMU 时间同步标定鲁棒性实验")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else core.default_out_dir()

    print("=" * 72)
    print(f"鲁棒性实验：4 组参数 × {args.trials} 次蒙特卡洛，真值偏移 {D_TRUE_MS:.0f} ms")
    print("=" * 72)

    print("\n[1] 相机角速度噪声 (rad/s)")
    e_nz, _, r_nz = _sweep("cam_noise", CAM_NOISES, args.trials)
    for v, (m, s), r in zip(CAM_NOISES, e_nz, r_nz):
        print(f"    noise={v:.3f}   误差 {m:+6.2f} ± {s:.2f} ms   rho={r:.4f}")

    print("\n[2] IMU 陀螺常值零偏 (rad/s)")
    e_bi, _, r_bi = _sweep("imu_bias", IMU_BIASES, args.trials)
    for v, (m, s), r in zip(IMU_BIASES, e_bi, r_bi):
        print(f"    bias ={v:.3f}   误差 {m:+6.2f} ± {s:.2f} ms   rho={r:.4f}")

    print("\n[3] 相机侧角速度尺度误差（真值 1.0）")
    e_ga, s_ga, r_ga = _sweep("cam_gain", CAM_GAINS, args.trials)
    for v, (m, s), sest, r in zip(CAM_GAINS, e_ga, s_ga, r_ga):
        print(f"    gain ={v:.2f}   误差 {m:+6.2f} ± {s:.2f} ms   "
              f"尺度比估计 {sest:.3f}   rho={r:.4f}")

    print("\n[4] 运动激励幅度（真值角速度峰值 1.346 rad/s 的缩放比）")
    e_ex, _, r_ex = _sweep("excitation", EXCITATIONS, args.trials)
    for v, (m, s), r in zip(EXCITATIONS, e_ex, r_ex):
        print(f"    x{v:.2f} (|w|峰值 {1.346*v:.3f})  误差 {m:+6.2f} ± {s:.2f} ms   "
              f"rho={r:.4f}")

    print()
    print(f"  零偏 0 → 0.5 rad/s：|误差变化| = {abs(e_bi[-1,0]-e_bi[0,0]):.2f} ms（去均值使其免疫）")
    print(f"  尺度 0.9 → 1.3 倍：|误差变化| = {abs(e_ga[-1,0]-e_ga[0,0]):.2f} ms（与偏移解耦）")
    print(f"  激励降到 2%：误差 {e_ex[-1,0]:+.2f} ± {e_ex[-1,1]:.2f} ms、rho {r_ex[-1]:.3f}"
          f"（均值看着不大，但单次结果的散布已达 ±{e_ex[-1,1]:.1f} ms，不可用）")
    print()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.0))

    ax = axes[0][0]
    ax.errorbar(np.asarray(CAM_NOISES) * 1000.0, e_nz[:, 0], yerr=e_nz[:, 1],
                fmt="o-", markersize=4, linewidth=1.3, color="#2b6cb0",
                ecolor="#90cdf4", elinewidth=0.8, capsize=2)
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    core.style_axes(ax, "camera noise [mrad/s]", "offset error [ms]",
                    "(a) camera angular-rate noise")

    ax = axes[0][1]
    ax.errorbar(np.asarray(IMU_BIASES) * 1000.0, e_bi[:, 0], yerr=e_bi[:, 1],
                fmt="o-", markersize=4, linewidth=1.3, color="#276749",
                ecolor="#9ae6b4", elinewidth=0.8, capsize=2)
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    ax.set_ylim(-0.6, 0.6)
    core.style_axes(ax, "IMU gyro bias [mrad/s]", "offset error [ms]",
                    "(b) constant gyro bias: immune")

    ax = axes[1][0]
    ax.errorbar(CAM_GAINS, e_ga[:, 0], yerr=e_ga[:, 1], fmt="o-", markersize=4,
                linewidth=1.3, color="#b7791f", ecolor="#fbd38d",
                elinewidth=0.8, capsize=2, label="offset error")
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    core.style_axes(ax, "camera scale error (truth 1.0)", "offset error [ms]",
                    "(c) scale error: offset stays put")
    # 右轴：尺度比估计值，验证尺度误差被单独吸收而不是混进偏移
    ax3 = ax.twinx()
    ax3.plot(CAM_GAINS, s_ga, color="#2b6cb0", linewidth=1.0, linestyle="--",
             marker="s", markersize=3, label="estimated scale")
    ax3.set_ylabel("estimated scale ratio", fontsize=8, color="#2b6cb0")
    ax3.tick_params(labelsize=7, colors="#2b6cb0")

    ax = axes[1][1]
    ax.errorbar(EXCITATIONS, e_ex[:, 0], yerr=e_ex[:, 1], fmt="o-", markersize=4,
                linewidth=1.3, color="#c53030", ecolor="#feb2b2",
                elinewidth=0.8, capsize=2)
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    core.style_axes(ax, "excitation scale (weaker → stronger)", "offset error [ms]",
                    "(d) weak excitation degrades")
    ax.set_xscale("log")
    ax2 = ax.twinx()
    ax2.plot(EXCITATIONS, r_ex, color="#2b6cb0", linewidth=1.0,
             linestyle="--", marker="s", markersize=3)
    ax2.set_ylabel("peak correlation $\\rho$", fontsize=8, color="#2b6cb0")
    ax2.tick_params(labelsize=7, colors="#2b6cb0")

    core.savefig(fig, out_dir / "robustness.png")
    print(f"图片目录：{out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
