#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轨迹 / 里程计精度评估 —— 统一入口。

示例::

    python3 main.py --mode demo                       # 默认演示（figure8 + 2% 轮半径误差）
    python3 main.py --mode demo --kind circle --scale 1.05
    python3 main.py --mode eval --gt gt.csv --est est.csv
    python3 main.py --mode sweep                      # 扫描尺度误差 0.98~1.02
    python3 main.py --mode ros                        # 启动 ROS 1 在线评估

约定：入口文件名固定为 ``main.py``；不带参数时等价于 ``--mode demo``。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from traj_eval import (  # noqa: E402
    KINDS, OdomErrorModel, ate, drift_percent, evaluate_all, load_trajectory,
    make_ground_truth, rpe, save_trajectory, simulate_odometry,
)

DEFAULT_OUTPUT = os.path.join(_HERE, "results")


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="轨迹/里程计精度评估：ATE、RPE 与漂移率（纯 Python，不依赖 ROS 与仿真器）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--mode", default="demo", choices=["demo", "eval", "sweep", "ros"],
                   help="demo=生成示例并评估；eval=评估两个 CSV；sweep=尺度误差扫描；ros=启动 ROS 节点")
    p.add_argument("--kind", default="figure8", choices=list(KINDS), help="真值轨迹形状")
    p.add_argument("--speed", type=float, default=1.5, help="行驶速度 m/s")
    p.add_argument("--duration", type=float, default=40.0, help="仿真时长 s")
    p.add_argument("--dt", type=float, default=0.05, help="采样间隔 s")
    p.add_argument("--scale", type=float, default=1.02, help="轮半径尺度误差（1.02 表示多算 2%%）")
    p.add_argument("--yaw-bias", type=float, default=0.0, help="每米航向偏置（度/米）")
    p.add_argument("--noise", type=float, default=0.0, help="单步平移噪声标准差 m")
    p.add_argument("--gt", default="", help="eval 模式的真值轨迹 CSV")
    p.add_argument("--est", default="", help="eval 模式的估计轨迹 CSV")
    p.add_argument("--output", default=DEFAULT_OUTPUT, help="结果输出目录")
    p.add_argument("--no-figures", action="store_true", help="只输出数值，不生成图表")
    return p.parse_args(argv)


def _print_metrics(title: str, metrics: dict) -> None:
    print(f"\n{title}")
    print("-" * 62)
    print(f"  轨迹长度            : {metrics['gt_path_length_m']:.2f} m")
    print(f"  ATE RMSE            : {metrics['ate_rmse_m']:.4f} m"
          f"  (mean {metrics['ate_mean_m']:.4f} / max {metrics['ate_max_m']:.4f})")
    print(f"  ATE 朝向 RMSE       : {metrics['ate_rmse_rot_deg']:.3f} °")
    print(f"  对齐尺度 / 旋转      : {metrics['ate_scale']:.4f} / {metrics['ate_yaw_offset_deg']:.2f} °")
    delta = metrics.get("rpe_delta")
    rpe_keys = sorted(int(k.split("_")[1]) for k in metrics if k.startswith("rpe_") and k.endswith("_count"))
    for d in rpe_keys:
        print(f"  RPE(delta={d}) 平移 : "
              f"RMSE {metrics[f'rpe_{d}_trans_rmse_m']:.4f} m / "
              f"mean {metrics[f'rpe_{d}_trans_mean_m']:.4f} m")
        print(f"  RPE(delta={d}) 旋转 : "
              f"RMSE {metrics[f'rpe_{d}_rot_rmse_deg']:.3f} ° / "
              f"mean {metrics[f'rpe_{d}_rot_mean_deg']:.3f} °")
    print(f"  终点漂移率(原始)    : {metrics['drift_percent']:.3f} %")
    print(f"  终点漂移率(对齐后)  : {metrics['drift_percent_aligned']:.3f} %")


def run_demo(args) -> int:
    print("=" * 62)
    print("  轨迹/里程计精度评估 —— 演示")
    print("=" * 62)
    print(f"真值轨迹: {args.kind} | 速度 {args.speed} m/s | 时长 {args.duration} s")
    print(f"误差模型: 尺度系数 {args.scale} | 航向偏置 {args.yaw_bias} °/m | 噪声 σ={args.noise} m")

    gt = make_ground_truth(args.kind, speed=args.speed, duration=args.duration, dt=args.dt)
    model = OdomErrorModel(wheel_scale=args.scale, yaw_bias_deg_per_m=args.yaw_bias,
                           trans_noise_std=args.noise, seed=0)
    est = simulate_odometry(gt, model)

    metrics = evaluate_all(est, gt, deltas=(1, 10, 50))
    _print_metrics("评估结果（ATE 默认做刚体对齐，不吸收尺度误差）", metrics)

    os.makedirs(args.output, exist_ok=True)
    gt_csv = save_trajectory(os.path.join(args.output, "ground_truth.csv"), gt, dt=args.dt)
    est_csv = save_trajectory(os.path.join(args.output, "odometry.csv"), est, dt=args.dt)
    with open(os.path.join(args.output, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, ensure_ascii=False, indent=2)
    print(f"\n已保存: {gt_csv}")
    print(f"已保存: {est_csv}")

    if not args.no_figures:
        try:
            from traj_eval import viz
            print("已生成:", viz.plot_trajectories(gt, est, os.path.join(args.output, "trajectory.png")))
            print("已生成:", viz.plot_error_curve(gt, est, os.path.join(args.output, "error.png"),
                                                  times=[i * args.dt for i in range(len(gt))]))
        except RuntimeError as exc:
            print(f"跳过绘图：{exc}")

    print("\n量化结论：")
    print(f"  轮半径尺度误差 {100 * (args.scale - 1):+.2f}% → 跑 {metrics['gt_path_length_m']:.1f} m "
          f"累计终点偏差 {drift_percent(est, gt, align=False):.2f}%（未对齐的原始坐标）")
    return 0


def run_eval(args) -> int:
    if not args.gt or not args.est:
        raise SystemExit("eval 模式需要 --gt 与 --est 两个 CSV")
    gt = load_trajectory(args.gt)
    est = load_trajectory(args.est)
    if len(gt) != len(est):
        raise SystemExit(f"两条轨迹长度不一致: gt={len(gt)} est={len(est)}")
    metrics = evaluate_all(est, gt, deltas=(1, 10, 50))
    _print_metrics(f"{args.gt}  vs  {args.est}", metrics)
    os.makedirs(args.output, exist_ok=True)
    out = os.path.join(args.output, "metrics.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(metrics, fh, ensure_ascii=False, indent=2)
    print(f"\n指标已保存: {out}")
    return 0


def run_sweep(args) -> int:
    gt = make_ground_truth(args.kind, speed=args.speed, duration=args.duration, dt=args.dt)
    scales = [0.96, 0.98, 0.99, 1.00, 1.01, 1.02, 1.04]
    rows = []
    print("=" * 84)
    print("  轮半径尺度误差 → 里程计精度敏感性")
    print("=" * 84)
    print(f"{'尺度系数':>10}{'尺度误差':>12}{'ATE RMSE/m':>14}{'ATE max/m':>12}{'漂移率/%':>12}")
    print("-" * 84)
    for s in scales:
        est = simulate_odometry(gt, OdomErrorModel(wheel_scale=s))
        a = ate(est, gt, align=True)
        d = drift_percent(est, gt, align=False)
        rows.append({"scale": s, "ate_rmse": a.rmse, "ate_max": a.max, "drift_percent": d})
        print(f"{s:>10.3f}{100 * (s - 1):>11.2f}%{a.rmse:>14.4f}{a.max:>12.4f}{d:>12.3f}")

    os.makedirs(args.output, exist_ok=True)
    with open(os.path.join(args.output, "sweep_scale.json"), "w", encoding="utf-8") as fh:
        json.dump(rows, fh, ensure_ascii=False, indent=2)
    if not args.no_figures:
        try:
            from traj_eval import viz
            print("\n已生成:", viz.plot_sweep(scales, [r["drift_percent"] for r in rows],
                                             [r["ate_rmse"] for r in rows],
                                             os.path.join(args.output, "sweep_scale.png")))
        except RuntimeError as exc:
            print(f"跳过绘图：{exc}")
    print("\n结论：终点漂移率随尺度误差近似线性增长——这正是需要标定轮半径的原因。")
    return 0


def run_ros(args) -> int:
    try:
        from traj_eval import ros_node
    except ImportError as exc:
        raise SystemExit(f"无法导入 ROS 依赖（{exc}）。请先 source /opt/ros/noetic/setup.bash") from exc
    ros_node.main()
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.mode == "demo":
        return run_demo(args)
    if args.mode == "eval":
        return run_eval(args)
    if args.mode == "sweep":
        return run_sweep(args)
    if args.mode == "ros":
        return run_ros(args)
    raise SystemExit(f"未知模式 {args.mode}")


if __name__ == "__main__":
    sys.exit(main())
