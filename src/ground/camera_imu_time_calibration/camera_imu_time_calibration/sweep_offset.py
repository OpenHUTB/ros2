#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""偏移量扫描：真值偏移从 -200 ms 到 +200 ms，验证估计器无系统性偏差。

    cd src/ground/camera_imu_time_calibration
    python3 -m camera_imu_time_calibration.sweep_offset --trials 20
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from camera_imu_time_calibration import core


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="相机-IMU 时间偏移量扫描")
    parser.add_argument("--trials", type=int, default=20, help="每个偏移取多少个随机种子")
    parser.add_argument("--step", type=float, default=20.0, help="偏移扫描步长 ms")
    parser.add_argument("--cam-rate", type=float, default=30.0)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else core.default_out_dir()
    offsets_ms = np.arange(-200.0, 200.0 + 1e-9, args.step)

    print("=" * 68)
    print(f"偏移量扫描：{len(offsets_ms)} 个偏移 × {args.trials} 次蒙特卡洛")
    print("=" * 68)
    print(f"相机 {args.cam_rate:.0f} Hz，噪声 0.010 rad/s，时长 20 s，细搜步长 0.1 ms")
    print()

    mean_est, std_est, mean_err, std_err = [], [], [], []
    for om in offsets_ms:
        est = []
        for k in range(args.trials):
            _, res = core.trial(seed=k, offset=om / 1000.0, cam_rate=args.cam_rate)
            est.append(res["offset"] * 1000.0)
        est = np.asarray(est)
        err = est - om
        mean_est.append(est.mean())
        std_est.append(est.std())
        mean_err.append(err.mean())
        std_err.append(err.std())
        if abs(om % 100.0) < 1e-9 or om == 0.0:
            print(f"  真值 {om:+7.1f} ms → 估计 {est.mean():+8.2f} ± {est.std():.2f} ms"
                  f"   误差 {err.mean():+6.2f} ms（std {err.std():.2f}）")

    mean_est = np.asarray(mean_est)
    std_est = np.asarray(std_est)
    mean_err = np.asarray(mean_err)
    std_err = np.asarray(std_err)

    # 线性拟合，检查是否接近斜率 1、截距 0
    slope, intercept = np.polyfit(offsets_ms, mean_est, 1)
    print()
    print(f"  线性拟合：估计 = {slope:.4f} × 真值 + {intercept:.3f} ms")
    print(f"  全范围最大 |误差| = {np.abs(mean_err).max():.2f} ms，"
          f"平均 |误差| = {np.abs(mean_err).mean():.2f} ms")
    print(f"  最大重复性 std = {std_err.max():.2f} ms（蒙特卡洛 {args.trials} 次）")
    print()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2))
    ax = axes[0]
    lim = [-220, 220]
    ax.plot(lim, lim, color="#a0aec0", linewidth=1.0, linestyle="--",
            label="$y = x$")
    ax.errorbar(offsets_ms, mean_est, yerr=std_est, fmt="o-", markersize=3,
                linewidth=1.2, color="#2b6cb0", ecolor="#90cdf4",
                elinewidth=0.8, capsize=0, label="estimate (mean ± std)")
    core.style_axes(ax, "true offset [ms]", "estimated offset [ms]",
                    "Identity check")
    ax.set_xlim(lim)
    ax.set_ylim(lim)
    ax.legend(fontsize=7, loc="upper left")

    ax = axes[1]
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    ax.fill_between(offsets_ms, mean_err - std_err, mean_err + std_err,
                    color="#90cdf4", alpha=0.5, linewidth=0, label="±1 std")
    ax.plot(offsets_ms, mean_err, "o-", markersize=3, linewidth=1.2,
            color="#2b6cb0", label="mean error")
    core.style_axes(ax, "true offset [ms]", "estimation error [ms]",
                    "Residual error across the range")
    ax.legend(fontsize=7, loc="upper left")
    core.savefig(fig, out_dir / "offset_sweep.png")
    print(f"图片目录：{out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
