#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""帧率、时间戳约定与曝光时长的扫描。

三个问题：

1. 相机帧率降到 10 Hz，偏移估计还准吗？
2. 如果时间戳打在**帧开始**而不是曝光中点，会引入多大的固定偏差？
3. 曝光时长**不等于**帧周期时，这个偏差还是"半帧"吗？

第 3 个问题的答案是否定的：等效测量时刻是 ``t_stamp + exposure/2``，所以偏差是
``-exposure/2``；只有 ``exposure`` 恰好等于帧周期时才退化为半帧。而时间戳打在
**曝光中点**时窗口对称，偏差与曝光时长无关。

    cd src/ground/camera_imu_time_calibration
    python3 -m camera_imu_time_calibration.sweep_rate --trials 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from camera_imu_time_calibration import core

RATES = (10.0, 15.0, 20.0, 30.0, 60.0, 120.0)
D_TRUE_MS = 50.0

# 曝光时长扫描：固定在 30 Hz，让"曝光 / 帧周期"从 10% 到 100% 变化
EXPOSURE_RATE = 30.0
EXPOSURE_RATIOS = (0.1, 0.25, 0.5, 0.75, 1.0)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="相机帧率、时间戳约定与曝光时长扫描")
    parser.add_argument("--trials", type=int, default=10, help="每种配置的随机种子数")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    out_dir = Path(args.out_dir) if args.out_dir else core.default_out_dir()

    print("=" * 84)
    print(f"帧率扫描：{len(RATES)} 档 × {args.trials} 次，真值偏移 {D_TRUE_MS:.0f} ms"
          f"（曝光 = 帧周期，即整帧窗口平均）")
    print("=" * 84)
    print(f"{'帧率[Hz]':>9}{'帧数':>7}{'mid 误差[ms]':>16}{'start 偏差[ms]':>17}"
          f"{'理论半帧[ms]':>15}{'补偿后姿态RMS[°]':>18}")

    err_mid_m, err_mid_s, bias_start, theory, att_rms = [], [], [], [], []
    for rate in RATES:
        e_mid, e_start, e_att = [], [], []
        for k in range(args.trials):
            sim, res = core.trial(seed=k, offset=D_TRUE_MS / 1000.0, cam_rate=rate)
            e_mid.append((res["offset"] - D_TRUE_MS / 1000.0) * 1000.0)
            e_att.append(core.attitude_error(res["offset"], sim)["rms_deg"])
            _, res_s = core.trial(seed=k, offset=D_TRUE_MS / 1000.0, cam_rate=rate,
                                  timestamp_at="start")
            e_start.append((res_s["offset"] - D_TRUE_MS / 1000.0) * 1000.0)
        n_cam = int(np.floor(sim["duration"] * rate))
        err_mid_m.append(float(np.mean(e_mid)))
        err_mid_s.append(float(np.std(e_mid)))
        bias_start.append(float(np.mean(e_start)))
        theory.append(-500.0 / rate)          # 半帧 = T/2 = 1/(2 f)
        att_rms.append(float(np.mean(e_att)))
        print(f"{rate:>9.0f}{n_cam:>7d}{np.mean(e_mid):>9.2f}±{np.std(e_mid):<6.2f}"
              f"{np.mean(e_start):>17.2f}{theory[-1]:>15.2f}{att_rms[-1]:>18.2f}")

    print()
    print(f"  mid 模式最大 |误差| = {np.abs(err_mid_m).max():.2f} ms"
          f"（帧率从 120 Hz 降到 10 Hz 误差没有明显恶化）")
    print(f"  start 模式实测偏差与理论半帧的最大差 = "
          f"{np.abs(np.asarray(bias_start) - np.asarray(theory)).max():.2f} ms")
    print(f"  补偿后姿态 RMS 误差随帧率升高而下降：10 Hz {att_rms[0]:.2f}° → "
          f"120 Hz {att_rms[-1]:.2f}°（离散化底噪）")
    print()

    # --- 曝光时长扫描：偏差跟的是 exposure/2，不是帧周期的 1/2 ---
    frame_period_ms = 1000.0 / EXPOSURE_RATE
    print("=" * 84)
    print(f"曝光时长扫描：{EXPOSURE_RATE:.0f} Hz（帧周期 {frame_period_ms:.2f} ms）× "
          f"{args.trials} 次，真值偏移 {D_TRUE_MS:.0f} ms")
    print("=" * 84)
    print(f"{'曝光/帧周期':>12}{'曝光[ms]':>10}{'start 偏差[ms]':>16}"
          f"{'理论 -Texp/2':>15}{'mid 误差[ms]':>15}")

    exp_ratio, exp_bias_start, exp_theory, exp_err_mid = [], [], [], []
    for ratio in EXPOSURE_RATIOS:
        exposure = ratio * frame_period_ms / 1000.0
        e_start, e_mid = [], []
        for k in range(args.trials):
            _, r_s = core.trial(seed=k, offset=D_TRUE_MS / 1000.0,
                                cam_rate=EXPOSURE_RATE, timestamp_at="start",
                                exposure=exposure)
            e_start.append((r_s["offset"] - D_TRUE_MS / 1000.0) * 1000.0)
            _, r_m = core.trial(seed=k, offset=D_TRUE_MS / 1000.0,
                                cam_rate=EXPOSURE_RATE, exposure=exposure)
            e_mid.append((r_m["offset"] - D_TRUE_MS / 1000.0) * 1000.0)
        exp_ratio.append(ratio * 100.0)
        exp_bias_start.append(float(np.mean(e_start)))
        exp_theory.append(-0.5 * ratio * frame_period_ms)
        exp_err_mid.append(float(np.mean(e_mid)))
        print(f"{ratio*100:>11.0f}%{ratio*frame_period_ms:>10.2f}"
              f"{np.mean(e_start):>16.2f}{exp_theory[-1]:>15.2f}{np.mean(e_mid):>15.2f}")

    print()
    print(f"  start 偏差与 -Texp/2 的最大差 = "
          f"{np.abs(np.asarray(exp_bias_start) - np.asarray(exp_theory)).max():.2f} ms"
          f" → 偏差由**曝光时长**决定，帧周期只在 Texp = T 时才是同一个数")
    print(f"  mid 误差在全部曝光时长下最大 |误差| = "
          f"{np.abs(np.asarray(exp_err_mid)).max():.2f} ms → 对称窗口与曝光长短无关")
    print()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rates = np.asarray(RATES)
    fig, axes = plt.subplots(1, 3, figsize=(10.4, 3.2))
    ax = axes[0]
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle="--")
    ax.errorbar(rates, err_mid_m, yerr=err_mid_s, fmt="o-", markersize=4,
                linewidth=1.3, color="#2b6cb0", ecolor="#90cdf4",
                elinewidth=0.8, capsize=2,
                label="timestamp at mid-exposure")
    core.style_axes(ax, "camera frame rate [Hz]", "offset error [ms]",
                    "Estimation error vs frame rate")
    ax.set_xscale("log")
    ax.set_xticks(rates)
    ax.set_xticklabels([f"{r:.0f}" for r in rates])
    ax.legend(fontsize=7, loc="upper right")

    ax = axes[1]
    ax.plot(rates, theory, color="#a0aec0", linewidth=1.2, linestyle="--",
            label="theory $-T/2$")
    ax.plot(rates, bias_start, "o-", markersize=4, linewidth=1.3,
            color="#c53030", label="measured bias")
    core.style_axes(ax, "camera frame rate [Hz]", "offset bias [ms]",
                    "Bias at frame start, $T_{exp}=T$")
    ax.set_xscale("log")
    ax.set_xticks(rates)
    ax.set_xticklabels([f"{r:.0f}" for r in rates])
    ax.legend(fontsize=7, loc="lower right")

    ax = axes[2]
    ax.plot(exp_ratio, exp_theory, color="#a0aec0", linewidth=1.2,
            linestyle="--", label="theory $-T_{exp}/2$")
    ax.plot(exp_ratio, exp_bias_start, "o-", markersize=4, linewidth=1.3,
            color="#c53030", label="frame-start bias")
    ax.plot(exp_ratio, exp_err_mid, "s-", markersize=4, linewidth=1.3,
            color="#2b6cb0", label="mid-exposure error")
    core.style_axes(ax, "exposure / frame period [%]", "offset bias [ms]",
                    f"Bias vs exposure, {EXPOSURE_RATE:.0f} Hz")
    ax.axhline(0.0, color="#a0aec0", linewidth=1.0, linestyle=":")
    ax.legend(fontsize=7, loc="lower left")

    core.savefig(fig, out_dir / "rate_sweep.png")
    print(f"图片目录：{out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
