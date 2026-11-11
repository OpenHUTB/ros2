"""地面载具横向控制算法对比：纯跟踪 / Stanley / LQR。

用法
----
python main.py --mode bench
python main.py --mode single --scenario sine
python main.py --mode sweep
python main.py --mode bench --model dynamic --speed 8
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

from src.controllers import make_controllers
from src.metrics import format_summary, rank_records
from src.path import make_scenarios
from src.plotting import (
    plot_arc_trajectories,
    plot_error_curves,
    plot_metrics,
    plot_single,
    plot_sweep,
)
from src.simulator import results_to_records, run_benchmark, simulate
from src.vehicle import VehicleParams

for _stream in (sys.stdout, sys.stderr):
    _reconfigure = getattr(_stream, "reconfigure", None)
    if _reconfigure is not None:
        _reconfigure(encoding="utf-8", errors="replace")

PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_ROOT / "results"

CSV_FIELDS = [
    "scenario", "scenario_title", "controller", "label", "model",
    "mean_abs", "rms", "max_abs", "steady_abs", "convergence_s",
    "offroad_ratio", "steer_rate_rms", "final_abs", "metric_rank", "scenario_rank",
]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Kinematic ground-vehicle lateral control benchmark (numpy only)"
    )
    parser.add_argument("--mode", choices=("bench", "single", "sweep"), default="bench")
    parser.add_argument("--model", choices=("kinematic", "dynamic"), default="kinematic")
    parser.add_argument("--speed", type=float, default=8.0, help="cruise speed [m/s]")
    parser.add_argument("--scenario", default="sine",
                        choices=("straight", "arc", "sine", "double_lane"))
    parser.add_argument("--controller", default="all",
                        choices=("all", "pure_pursuit", "stanley", "lqr"))
    parser.add_argument("--dt", type=float, default=0.02)
    return parser.parse_args(argv)


def _write_csv(path, rows, fieldnames):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _print_table(ranked):
    print("\n{:14} {:12} {:>8} {:>8} {:>8} {:>9} {:>7} {:>8} {:>6}".format(
        "Controller", "Scenario", "Mean|e|", "RMS", "Max|e|", "Steady", "Conv[s]", "Offroad", "Rank"))
    print("-" * 94)
    for item in ranked:
        print("{:14} {:12} {:8.4f} {:8.4f} {:8.4f} {:9.4f} {:7.2f} {:8.3f} {:6.2f}".format(
            item["label"], item["scenario"], item["mean_abs"], item["rms"],
            item["max_abs"], item["steady_abs"], item["convergence_s"],
            item["offroad_ratio"], item["metric_rank"]))


def _print_ranking(summary):
    print("\n综合排名（场景内 7 项指标平均名次，再跨场景平均）:")
    for item in summary:
        print("  {}. {:<16} {:.3f}".format(item["overall_rank"], item["label"], item["mean_rank"]))


def _conclusion_text(ranked, summary):
    """根据本次实际结果生成简短中文结论，避免 README 与结果脱节。"""
    by_name = {}
    for item in ranked:
        by_name.setdefault(item["controller"], []).append(item)
    fastest = min(
        by_name.values(),
        key=lambda items: np.mean([i["convergence_s"] for i in items]),
    )
    lines = ["", "关键结论（由本次运行自动生成）", "-" * 78]
    lines.append("综合名次：")
    for item in summary:
        lines.append("  {}. {:<16} 平均名次 {:.3f}".format(
            item["overall_rank"], item["label"], item["mean_rank"]))
    fastest_label = fastest[0]["label"]
    fastest_conv = float(np.mean([i["convergence_s"] for i in fastest]))
    lines.append("收敛最快：{}（四场景平均 {:.2f} s）。".format(
        fastest_label, fastest_conv))
    avg_steady = {name: float(np.mean([i["steady_abs"] for i in items]))
                  for name, items in by_name.items()}
    best_steady = min(avg_steady, key=avg_steady.get)
    lines.append(
        "四场景平均稳态误差：{}（{:.5f} m）；LQR {:.5f} m，纯跟踪 {:.5f} m。".format(
            next(i["label"] for i in summary if i["controller"] == best_steady),
            avg_steady[best_steady], avg_steady["lqr"],
            avg_steady["pure_pursuit"]))
    for scenario in ("straight", "arc", "sine", "double_lane"):
        subset = [i for i in ranked if i["scenario"] == scenario]
        best = min(subset, key=lambda i: i["steady_abs"])
        lines.append("{} 稳态误差最小：{}（{:.4f} m）。".format(
            scenario, best["label"], best["steady_abs"]))
    pp_sine = next(i for i in ranked
                   if i["controller"] == "pure_pursuit" and i["scenario"] == "sine")
    stanley_arc = next(i for i in ranked
                       if i["controller"] == "stanley" and i["scenario"] == "arc")
    lqr_double = next(i for i in ranked
                      if i["controller"] == "lqr" and i["scenario"] == "double_lane")
    lines.append("纯跟踪在正弦连续变曲率上的稳态残差为 {:.4f} m；".format(
        pp_sine["steady_abs"]))
    lines.append("Stanley 后轴在圆弧上的稳态误差为 {:.5f} m；".format(
        stanley_arc["steady_abs"]))
    lines.append("LQR 在双移线上的平均绝对误差为 {:.4f} m、收敛 {:.2f} s。".format(
        lqr_double["mean_abs"], lqr_double["convergence_s"]))
    return "\n".join(lines) + "\n"


def run_bench(args):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    params = VehicleParams()
    controllers = make_controllers(params)
    scenarios = make_scenarios()
    results = run_benchmark(
        controllers, scenarios=scenarios, model=args.model,
        cruise_speed=args.speed, dt=args.dt,
    )
    records = results_to_records(results)
    ranked, summary = rank_records(records)

    _write_csv(RESULTS_DIR / "bench.csv", ranked, CSV_FIELDS)
    _write_csv(
        RESULTS_DIR / "ranking.csv", summary,
        ["overall_rank", "controller", "label", "mean_rank", "scenarios"],
    )
    _write_csv(
        RESULTS_DIR / "raw_metrics.csv", ranked,
        ["scenario", "controller", "mean_abs", "rms", "max_abs", "steady_abs",
         "convergence_s", "offroad_ratio", "steer_rate_rms"],
    )

    scenario_rows = {scenario.key: row for scenario, row in zip(scenarios.values(), results)}
    plot_metrics(ranked, RESULTS_DIR / "metrics.png")
    plot_arc_trajectories(scenario_rows["arc"], scenarios["arc"], RESULTS_DIR / "trajectory_arc.png")
    plot_error_curves(scenario_rows["sine"], scenarios["sine"], RESULTS_DIR / "error_sine.png")

    summary_text = format_summary(
        ranked, summary,
        title=f"Lateral Control Benchmark ({args.model}, cruise {args.speed:g} m/s)",
    )
    summary_text += _conclusion_text(ranked, summary)
    (RESULTS_DIR / "summary.txt").write_text(summary_text, encoding="utf-8")

    _print_table(ranked)
    _print_ranking(summary)
    print("\n输出目录:", RESULTS_DIR)
    print("已生成: metrics.png, trajectory_arc.png, error_sine.png, bench.csv, ranking.csv, summary.txt")
    return 0


def run_single(args):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    params = VehicleParams()
    scenarios = make_scenarios()
    scenario = scenarios[args.scenario]
    controllers = make_controllers(params)
    if args.controller != "all":
        controllers = [c for c in controllers if c.name == args.controller]
    results = [
        simulate(scenario, controller, model=args.model,
                 cruise_speed=args.speed, dt=args.dt)
        for controller in controllers
    ]
    tag = args.scenario
    trajectory_path = RESULTS_DIR / f"single_trajectory_{tag}.png"
    error_path = RESULTS_DIR / f"single_error_{tag}.png"
    plot_single(scenario, results, trajectory_path, error_path)

    rows = []
    for result in results:
        row = {"scenario": result.scenario, "scenario_title": result.scenario_title,
               "controller": result.controller, "label": result.label, "model": result.model}
        row.update(result.metrics)
        rows.append(row)
    _write_csv(
        RESULTS_DIR / f"single_metrics_{tag}.csv", rows,
        ["scenario", "scenario_title", "controller", "label", "model",
         "mean_abs", "rms", "max_abs", "steady_abs", "convergence_s",
         "offroad_ratio", "steer_rate_rms", "final_abs"],
    )
    print("\n单场景:", scenario.title)
    for row in rows:
        print("  {:<14} mean={:.4f} rms={:.4f} max={:.4f} steady={:.4f} conv={:.2f}s".format(
            row["label"], row["mean_abs"], row["rms"], row["max_abs"],
            row["steady_abs"], row["convergence_s"]))
    print("已生成:", trajectory_path.name, error_path.name)
    return 0


def run_sweep(args):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    params = VehicleParams()
    scenarios = make_scenarios()
    speeds = (4.0, 6.0, 8.0, 10.0, 12.0)
    rows = []
    for speed in speeds:
        controller_results = {c.name: [] for c in make_controllers(params)}
        for scenario in scenarios.values():
            for controller in make_controllers(params):
                result = simulate(scenario, controller, model=args.model,
                                  cruise_speed=speed, dt=args.dt)
                controller_results[controller.name].append(result)
        for name, items in controller_results.items():
            rows.append({
                "speed": speed,
                "controller": name,
                "label": items[0].label,
                "model": args.model,
                "mean_abs": float(np.mean([r.metrics["mean_abs"] for r in items])),
                "rms": float(np.mean([r.metrics["rms"] for r in items])),
                "convergence_s": float(np.mean([r.metrics["convergence_s"] for r in items])),
            })
        print("sweep: {:.0f} m/s done".format(speed))

    _write_csv(
        RESULTS_DIR / "sweep.csv", rows,
        ["speed", "controller", "label", "model", "mean_abs", "rms", "convergence_s"],
    )
    plot_sweep(rows, RESULTS_DIR / "sweep.png")
    print("已生成: sweep.csv, sweep.png")
    return 0


def main(argv=None):
    args = parse_args(argv)
    if args.speed <= 0.0:
        print("--speed must be positive", file=sys.stderr)
        return 2
    if args.dt <= 0.0:
        print("--dt must be positive", file=sys.stderr)
        return 2
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    try:
        if args.mode == "bench":
            return run_bench(args)
        if args.mode == "single":
            return run_single(args)
        return run_sweep(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report a clean failure
        print("运行失败: {}: {}".format(type(exc).__name__, exc), file=sys.stderr)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
