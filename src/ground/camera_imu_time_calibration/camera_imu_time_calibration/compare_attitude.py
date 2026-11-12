#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""补偿前后姿态误差对比：回答"这点毫秒级偏移到底有多要紧"。

给出三条曲线：

* **未补偿**：直接用相机角速度积分，忽略时间偏移；
* **实测补偿**：用估计出的偏移对齐后再积分；
* **解析估计**：$d \\cdot \\max_t |\\omega(t) - \\omega(0)|$，说明误差量级从哪来。

    cd src/ground/camera_imu_time_calibration
    python3 -m camera_imu_time_calibration.compare_attitude --trials 5
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from camera_imu_time_calibration import core

OFFSETS_MS = (0.0, 5.0, 10.0, 20.0, 30.0, 50.0, 80.0, 120.0, 200.0)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="补偿前后姿态误差对比")
    parser.add_argument("--trials", type=int, default=5, help="每个偏移的随机种子数")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else core.default_out_dir()

    # 解析项的常数：max_t |w(t) - w(0)|
    t_ref = np.linspace(0.0, 20.0, 20001)
    span = float(np.linalg.norm(core.omega(t_ref) - core.omega(0.0), axis=1).max())
    print("=" * 76)
    print("补偿前后姿态误差对比")
    print("=" * 76)
    print(f"角速度峰值 |w| = {core.omega_peak():.3f} rad/s，"
          f"max|w(t)-w(0)| = {span:.3f} rad/s")
    print(f"解析估计：最大姿态误差 ≈ d × max|w(t)-w(0)| "
          f"= {np.degrees(span):.1f}° 每 1000 ms")
    print()

    print(f"{'偏移[ms]':>9}{'未补偿 max':>12}{'未补偿 RMS':>12}"
          f"{'补偿后 max':>12}{'补偿后 RMS':>12}{'解析 max':>11}{'下降':>9}")

    raw_max, raw_rms, fix_max, fix_rms, ana = [], [], [], [], []
    for om in OFFSETS_MS:
        m0, r0, m1, r1 = [], [], [], []
        for k in range(args.trials):
            sim, res = core.trial(seed=k, offset=om / 1000.0)
            e0 = core.attitude_error(0.0, sim)
            e1 = core.attitude_error(res["offset"], sim)
            m0.append(e0["max_deg"])
            r0.append(e0["rms_deg"])
            m1.append(e1["max_deg"])
            r1.append(e1["rms_deg"])
        raw_max.append(np.mean(m0))
        raw_rms.append(np.mean(r0))
        fix_max.append(np.mean(m1))
        fix_rms.append(np.mean(r1))
        ana.append(np.degrees(span * om / 1000.0))
        drop = 100.0 * (1.0 - np.mean(r1) / np.mean(r0))
        print(f"{om:>9.0f}{np.mean(m0):>12.2f}{np.mean(r0):>12.2f}"
              f"{np.mean(m1):>12.2f}{np.mean(r1):>12.2f}{ana[-1]:>11.2f}{drop:>8.0f}%")

    print()
    print(f"  50 ms 偏移：未补偿姿态 RMS {raw_rms[5]:.2f}°，补偿后 {fix_rms[5]:.2f}°"
          f"（降低 {100*(1-fix_rms[5]/raw_rms[5]):.0f}%）")
    print(f"  补偿后的残差主要是 30 Hz 相机的离散化底噪，与偏移大小几乎无关")
    print(f"  解析项与实测未补偿值的比值（不含 0 ms）："
          f"{np.mean(np.asarray(raw_max[1:])/np.asarray(ana[1:])):.2f}"
          f"（1.0 附近说明量级估计可靠）")
    print()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2))
    offs = np.asarray(OFFSETS_MS)

    ax = axes[0]
    ax.plot(offs, raw_max, "o-", markersize=4, linewidth=1.4, color="#c53030",
            label="max, no compensation")
    ax.plot(offs, fix_max, "s-", markersize=4, linewidth=1.4, color="#2b6cb0",
            label="max, estimated offset")
    ax.plot(offs, ana, color="#a0aec0", linewidth=1.1, linestyle="--",
            label="analytic $d\\cdot\\max|\\omega(t)-\\omega(0)|$")
    core.style_axes(ax, "true offset [ms]", "max attitude error [deg]",
                    "Attitude error vs offset")
    ax.legend(fontsize=7, loc="upper left")

    ax = axes[1]
    ax.plot(offs, raw_rms, "o-", markersize=4, linewidth=1.4, color="#c53030",
            label="RMS, no compensation")
    ax.plot(offs, fix_rms, "s-", markersize=4, linewidth=1.4, color="#2b6cb0",
            label="RMS, estimated offset")
    core.style_axes(ax, "true offset [ms]", "RMS attitude error [deg]",
                    "Compensation removes the offset term")
    ax.legend(fontsize=7, loc="upper left")
    core.savefig(fig, out_dir / "attitude_error.png")
    print(f"图片目录：{out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
