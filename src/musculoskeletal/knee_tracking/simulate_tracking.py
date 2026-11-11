#!/usr/bin/env python3
"""离线演示：跑正弦 + 阶跃，输出 CSV / 指标，可选出图。

用法：
    python3 simulate_tracking.py               # 生成曲线与指标（有 matplotlib 时出 PNG）
    python3 simulate_tracking.py --no-render   # 只出 CSV 与指标
    python3 simulate_tracking.py --duration 20 # 自定义正弦段时长
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path

from mujoco_musculoskeletal_knee_tracking.config import load_config
from mujoco_musculoskeletal_knee_tracking.model import KneeSim
from mujoco_musculoskeletal_knee_tracking.simulation import (
    run_tracking,
    sine_metrics,
    step_metrics,
)

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"


def _fmt(value) -> str:
    return "未进入" if value is None else f"{float(value):.4f}"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="膝关节肌肉骨骼离线演示")
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--config", default=None)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    RESULTS.mkdir(parents=True, exist_ok=True)
    sim = KneeSim(cfg)

    sine = run_tracking(sim, cfg, "sine", duration=args.duration or cfg.sim.duration,
                        initial_deg=cfg.trajectory.bias_deg)
    sm = sine_metrics(sine)

    step_duration = cfg.trajectory.step_time + 6.0
    step = run_tracking(sim, cfg, "step", duration=step_duration,
                        initial_deg=cfg.trajectory.step_start_deg)
    tm = step_metrics(step, cfg)

    with open(RESULTS / "tracking.csv", "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["segment", "t_s", "ref_deg", "theta_deg", "error_deg",
                         "flex", "ext", "torque_nm"])
        for name, run in (("sine", sine), ("step", step)):
            for k in range(len(run.t)):
                writer.writerow([name, f"{run.t[k]:.4f}", f"{run.ref_deg[k]:.4f}",
                                 f"{run.theta_deg[k]:.4f}", f"{run.error_deg[k]:.4f}",
                                 f"{run.flex[k]:.4f}", f"{run.ext[k]:.4f}",
                                 f"{run.torque[k]:.4f}"])

    lines = [
        "膝关节肌肉骨骼仿真与轨迹跟踪（MuJoCo + Hill 肌肉）",
        "=" * 58,
        f"生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"参数：dt={cfg.sim.dt:g}s  kp={cfg.ctrl.kp:g}  kd={cfg.ctrl.kd:g}  "
        f"max_torque={cfg.ctrl.max_torque:g} N·m",
        f"单肌：F_max={cfg.muscle.f_max:g} N  r={cfg.muscle.moment_arm:g} m",
        "",
        "[正弦段]",
        f"  RMSE           = {sm['rmse_deg']:.4f} deg",
        f"  最大绝对误差    = {sm['max_abs_error_deg']:.4f} deg",
        "",
        "[阶跃段]",
        f"  超调           = {tm['overshoot_percent']:.4f} %",
        f"  调节时间(±2%)  = {_fmt(tm['settling_time_s'])}",
        f"  ±5° 首次进入   = {_fmt(tm['wide_band_entry_s'])}",
        f"  稳态误差       = {tm['steady_state_error_deg']:+.4f} deg",
        f"  峰值           = {tm['peak_deg']:.4f} deg",
        "",
        "[对照文档]",
        "  文档：RMSE 8.65°，最大 13.96°，超调 0%，稳态 −6.62°，±5° 0.22 s",
    ]
    (RESULTS / "tracking_metrics.txt").write_text("\n".join(lines) + "\n",
                                                  encoding="utf-8")

    if not args.no_render:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(10, 4))
            ax.plot(sine.t, sine.ref_deg, "--", label="reference")
            ax.plot(sine.t, sine.theta_deg, label="actual")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("knee angle [deg]")
            ax.set_title(f"sine tracking, RMSE={sm['rmse_deg']:.2f} deg")
            ax.legend()
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(RESULTS / "tracking_eval.png", dpi=140)
            plt.close(fig)

            fig, ax = plt.subplots(figsize=(10, 4))
            ax.plot(step.t, step.ref_deg, "--", label="reference")
            ax.plot(step.t, step.theta_deg, label="actual")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("knee angle [deg]")
            ax.set_title(f"step response, steady error={tm['steady_state_error_deg']:.2f} deg")
            ax.legend()
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(RESULTS / "tracking_step.png", dpi=140)
            plt.close(fig)
        except Exception as exc:  # noqa: BLE001
            print(f"[warn] 跳过绘图：{exc}")

    print("\n".join(lines))
    print(f"\n输出目录：{RESULTS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
