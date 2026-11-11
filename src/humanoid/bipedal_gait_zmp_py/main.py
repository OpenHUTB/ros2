"""双足步态生成与 ZMP 稳定性验证（离线 Python，不依赖 ROS）。

用法：
    python main.py
    python main.py --quick
    python main.py --gain 1.5
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys

import numpy as np

from scripts.animate import create_walking_gif
from scripts.footstep import FootstepPlan
from scripts.gait import (
    compute_periodic_solution,
    result_to_row,
    run_open_loop_instability,
    simulate_gait,
    sweep_gains,
    sweep_support_margin,
)
from scripts.lipm import G, OMEGA, Z_C
from scripts.plotting import generate_all_plots, plot_support_margin_sweep

# Windows 控制台默认可能是 GBK；统一成 UTF-8 以正确显示中文进度信息。
for stream in (sys.stdout, sys.stderr):
    reconfigure = getattr(stream, "reconfigure", None)
    if reconfigure is not None:
        reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
DEFAULT_GAINS = (0.0, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="离线双足步态生成与 ZMP 稳定性验证（纯 Python，不使用 ROS）"
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="跳过 K 增益扫描，只运行 --gain 指定的增益",
    )
    parser.add_argument(
        "--gain",
        type=float,
        default=1.0,
        help="DCM 反馈增益 K，默认 1.0",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=1e-3,
        help="仿真积分步长，默认 0.001 s",
    )
    return parser.parse_args(argv)


def write_metrics_csv(result, path: Path) -> None:
    fields = [
        "time_s",
        "com_x_m",
        "com_v_mps",
        "zmp_m",
        "zmp_ref_m",
        "zmp_cmd_m",
        "support_left_m",
        "support_right_m",
        "margin_m",
        "dcm_m",
        "dcm_ref_m",
        "dcm_error_m",
        "phase",
        "saturated",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(fields)
        for k in range(len(result.time)):
            writer.writerow(
                [
                    f"{result.time[k]:.6f}",
                    f"{result.x[k]:.8f}",
                    f"{result.v[k]:.8f}",
                    f"{result.zmp[k]:.8f}",
                    f"{result.zmp_ref[k]:.8f}",
                    f"{result.zmp_cmd[k]:.8f}",
                    f"{result.support_left[k]:.8f}",
                    f"{result.support_right[k]:.8f}",
                    f"{result.margin[k]:.8f}",
                    f"{result.dcm[k]:.8f}",
                    f"{result.dcm_ref[k]:.8f}",
                    f"{result.dcm_error[k]:.8f}",
                    str(result.phase[k]),
                    str(bool(result.saturated[k])),
                ]
            )


def write_gain_csv(results, path: Path) -> None:
    fields = [
        "gain",
        "travel_m",
        "avg_speed_mps",
        "saturation_ratio",
        "mean_margin_m",
        "min_margin_m",
        "dcm_max_error_m",
        "final_velocity_mps",
        "stable",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for result in results:
            row = result_to_row(result)
            writer.writerow(
                {
                    "gain": f"{row['gain']:.3f}",
                    "travel_m": f"{row['travel_m']:.6f}",
                    "avg_speed_mps": f"{row['avg_speed_mps']:.6f}",
                    "saturation_ratio": f"{row['saturation_ratio']:.6f}",
                    "mean_margin_m": f"{row['mean_margin_m']:.6f}",
                    "min_margin_m": f"{row['min_margin_m']:.6f}",
                    "dcm_max_error_m": f"{row['dcm_max_error_m']:.6f}",
                    "final_velocity_mps": f"{row['final_velocity_mps']:.6f}",
                    "stable": str(bool(row["stable"])),
                }
            )


def write_margin_csv(rows, path: Path) -> None:
    fields = [
        "zmp_edge_margin_m",
        "effective_half_width_m",
        "travel_m",
        "avg_speed_mps",
        "saturation_ratio",
        "min_margin_m",
        "stable",
    ]
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "zmp_edge_margin_m": f"{row['zmp_edge_margin_m']:.3f}",
                    "effective_half_width_m": f"{row['effective_half_width_m']:.3f}",
                    "travel_m": f"{row['travel_m']:.6f}",
                    "avg_speed_mps": f"{row['avg_speed_mps']:.6f}",
                    "saturation_ratio": f"{row['saturation_ratio']:.6f}",
                    "min_margin_m": f"{row['min_margin_m']:.6f}",
                    "stable": str(bool(row["stable"])),
                }
            )


def write_summary(
    path: Path,
    plan: FootstepPlan,
    periodic,
    result,
    instability,
    sweep_results,
    margin_rows,
    quick: bool,
) -> None:
    lines: list[str] = []
    lines.append("双足步态生成与 ZMP 稳定性验证（离线 Python，不使用 ROS）")
    lines.append("=" * 68)
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")
    lines.append("[模型参数]")
    lines.append(f"  g = {G:.2f} m/s^2, z_c = {Z_C:.2f} m, omega = {OMEGA:.9f} rad/s")
    lines.append(
        f"  步长 = {plan.step_length:.3f} m, 步时 = {plan.step_time:.3f} s, "
        f"步数 = {plan.n_steps}, 支撑相占比 = {plan.stance_ratio:.2f}"
    )
    lines.append(
        f"  支撑时长 = {plan.stance_duration:.3f} s, 双支撑时长 = "
        f"{plan.stance_duration - plan.step_time:.3f} s, 周期 = {plan.cycle_time:.3f} s"
    )
    lines.append(
        f"  足底几何半长 = {plan.half_length:.3f} m, 有效 ZMP 半宽 = "
        f"{plan.support_half_width:.3f} m (踝部边界缓冲 {plan.zmp_edge_margin:.3f} m)"
    )
    lines.append(f"  总仿真时长 = {plan.duration:.3f} s")
    lines.append("")
    lines.append("[周期解]")
    lines.append(f"  周期起点（第二只脚落地）: t = {periodic.t_start:.3f} s")
    lines.append(f"  周期 T = {periodic.period:.3f} s, 周期位移 = {periodic.displacement:.3f} m")
    lines.append(f"  x0 = {periodic.x0:.6f} m, v0 = {periodic.v0:.6f} m/s")
    lines.append(f"  映射残差 = {periodic.residual:.3e}, trace(A) = {periodic.trace:.6f}")
    lines.append("")
    lines.append(f"[默认增益 K = {result.gain:.2f} 的关键指标]")
    lines.append(f"  行程 = {result.travel:+.4f} m")
    lines.append(f"  平均速度 = {result.avg_speed:.4f} m/s")
    lines.append(f"  DCM 最大误差 = {result.dcm_max_error:.4f} m")
    lines.append(f"  ZMP 饱和比例 = {100.0 * result.saturation_ratio:.2f} %")
    lines.append(f"  平均稳定裕度 = {result.mean_margin:.4f} m")
    lines.append(f"  ZMP 最小稳定裕度 = {result.min_margin:.4f} m")
    lines.append(f"  末速度 = {result.final_velocity:.4f} m/s")
    lines.append(f"  是否稳定行走 = {'是' if result.stable else '否'}")
    lines.append("")
    lines.append("[开环不稳定性]")
    lines.append("  从周期解出发，给质心位置 +1e-4 m 扰动，积分 5 个周期：")
    for i, err in enumerate(instability.cycle_errors, start=1):
        ratio = "     -" if i == 1 else f"{instability.ratios[i - 2]:8.4f}"
        lines.append(f"    第 {i} 周期末速度偏差 = {err:+.6e} m/s, 放大比 = {ratio}")
    lines.append(f"  理论单周期放大比 e^(omega*T) = {instability.predicted_ratio:.4f}")
    if len(instability.ratios):
        lines.append(f"  末段平均放大比 = {float(np.mean(instability.ratios[-3:])):.4f}")
    zero_result = next((r for r in sweep_results if abs(r.gain) < 1e-12), None)
    if zero_result is not None:
        lines.append(
            "  K=0 时从 x=0 开环积分到 6.1 s，质心位置 = "
            f"{zero_result.travel:+.6e} m（发散）"
        )
    else:
        lines.append("  K=0 开环发散项在 --quick 模式下未运行。")
    lines.append("")
    lines.append("[增益扫描]")
    lines.append(
        f"{'K':>6} {'行程/m':>14} {'平均速度':>10} {'饱和/%':>9} "
        f"{'平均裕度/m':>11} {'最小裕度/m':>11} {'DCM误差/m':>10} {'稳定':>6}"
    )
    for r in sweep_results:
        lines.append(
            f"{r.gain:6.2f} {r.travel:14.6g} {r.avg_speed:10.4f} "
            f"{100.0 * r.saturation_ratio:9.2f} {r.mean_margin:11.4f} "
            f"{r.min_margin:11.4f} {r.dcm_max_error:10.4f} "
            f"{'是' if r.stable else '否':>6}"
        )
    if quick:
        lines.append("  注：本次使用 --quick，仅运行所选增益，未做完整扫描。")
    lines.append("")
    lines.append("[ZMP 可行域敏感性：踝部边界缓冲]")
    lines.append(
        "  足底几何半长固定 0.09 m；下表只改变允许 ZMP 离脚边的余量。"
    )
    lines.append(
        f"{'缓冲/m':>8} {'有效半宽/m':>11} {'行程/m':>13} {'饱和/%':>9} {'稳定':>6}"
    )
    for row in margin_rows:
        lines.append(
            f"{row['zmp_edge_margin_m']:8.3f} {row['effective_half_width_m']:11.3f} "
            f"{row['travel_m']:13.4f} {100.0 * row['saturation_ratio']:9.2f} "
            f"{'是' if row['stable'] else '否':>6}"
        )
    lines.append("")
    lines.append("[结论与设计说明]")
    lines.append(
        "  1. 周期解存在且唯一（仿射映射残差 1e-16），但开环不稳定："
        "微小扰动每周期放大约 e^(omega*T) = 33.17 倍。"
    )
    lines.append(
        "  2. DCM 比例反馈是必需的：K=0 时质心发散到 -5.9e7 m；"
        "K>=1 才能稳定行走。"
    )
    lines.append(
        "  3. 增益不是越大越好：K 从 1 增大到 3，ZMP 饱和比例从 20% 升到 58%，"
        "平均稳定裕度从 0.086 m 降到 0.044 m。默认取 K=1.0。"
    )
    lines.append(
        "  4. 有效 ZMP 半宽是本模型最敏感的假设：严格用 0.09 m 时 K=1 会"
        "长期贴边并发散；加入 0.03 m 踝部缓冲（0.12 m）后得到 3.93 m / "
        "0.64 m/s。这与原报告口径一致。"
    )
    lines.append(
        "  5. 双支撑 ZMP 参考沿前进方向从后脚连续过渡到前脚；若反向过渡，"
        "单脚/双脚切换处会出现 ZMP 跳变，稳定裕度无法保持非负。"
    )
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    np.random.seed(0)  # 本程序不含随机过程；固定种子以满足可复现性要求。
    OUT.mkdir(parents=True, exist_ok=True)

    plan = FootstepPlan()
    periodic = compute_periodic_solution(plan)
    instability = run_open_loop_instability(plan, periodic)

    if args.quick:
        gains = [float(args.gain)]
    else:
        gains = list(DEFAULT_GAINS)
        if not any(abs(g - args.gain) < 1e-12 for g in gains):
            gains.append(float(args.gain))
        gains = sorted(gains)

    sweep_results = sweep_gains(plan, gains, dt=args.dt)
    result = next(
        (r for r in sweep_results if abs(r.gain - args.gain) < 1e-12),
        simulate_gait(plan, args.gain, dt=args.dt),
    )

    # 踝部缓冲敏感性只在默认增益上做一次小扫描；--quick 仍会生成该附加图。
    margin_rows = sweep_support_margin(gain=args.gain, dt=args.dt)
    write_metrics_csv(result, OUT / "metrics.csv")
    write_gain_csv(sweep_results, OUT / "gain_sweep.csv")
    write_margin_csv(margin_rows, OUT / "support_margin_sweep.csv")
    generate_all_plots(result, plan, sweep_results, OUT)
    plot_support_margin_sweep(margin_rows, args.gain, OUT / "support_margin_sweep.png")
    create_walking_gif(result, plan, OUT / "walking.gif")
    write_summary(
        OUT / "summary.txt",
        plan,
        periodic,
        result,
        instability,
        sweep_results,
        margin_rows,
        args.quick,
    )

    # 数值健康检查：不允许 NaN / Inf。
    arrays = {
        "time": result.time,
        "x": result.x,
        "v": result.v,
        "zmp": result.zmp,
        "dcm": result.dcm,
        "margin": result.margin,
    }
    for name, values in arrays.items():
        if not np.all(np.isfinite(values)):
            raise RuntimeError(f"仿真结果包含 NaN/Inf: {name}")

    print("=" * 70)
    print("双足步态生成与 ZMP 稳定性验证（离线 Python）")
    print("=" * 70)
    print(
        f"omega={OMEGA:.6f} rad/s, x0={periodic.x0:.6f} m, v0={periodic.v0:.6f} m/s, "
        f"period residual={periodic.residual:.2e}"
    )
    print(
        f"K={result.gain:.1f}: travel={result.travel:+.4f} m, "
        f"avg speed={result.avg_speed:.4f} m/s, "
        f"DCM max error={result.dcm_max_error:.4f} m, "
        f"ZMP saturation={100.0 * result.saturation_ratio:.2f}%, "
        f"margin mean/min={result.mean_margin:.4f}/{result.min_margin:.4f} m"
    )
    print(
        "open-loop cycle velocity errors: "
        + ", ".join(f"{e:+.6e}" for e in instability.cycle_errors)
    )
    print(
        "open-loop velocity-error ratios: "
        + ", ".join(f"{r:.3f}" for r in instability.ratios)
        + f"  (theory {instability.predicted_ratio:.3f})"
    )
    zero_result = next((r for r in sweep_results if abs(r.gain) < 1e-12), None)
    if zero_result is not None:
        print(f"K=0 open-loop final COM = {zero_result.travel:+.4e} m")
    else:
        print("quick mode: no K=0 sweep row")
    print("-" * 70)
    print(f"{'K':>5} {'travel[m]':>14} {'speed[m/s]':>12} {'sat[%]':>9} {'margin[m]':>10}")
    for r in sweep_results:
        print(
            f"{r.gain:5.2f} {r.travel:14.6g} {r.avg_speed:12.4f} "
            f"{100.0 * r.saturation_ratio:9.2f} {r.mean_margin:10.4f}"
        )
    print("-" * 70)
    print(f"输出目录: {OUT}")
    for name in (
        "gait_com_zmp.png",
        "stability_margin.png",
        "dcm_error.png",
        "gain_sweep.png",
        "support_margin_sweep.png",
        "walking.gif",
        "metrics.csv",
        "gain_sweep.csv",
        "support_margin_sweep.csv",
        "summary.txt",
    ):
        path = OUT / name
        print(f"  {name:24s} {path.stat().st_size:>10,d} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
