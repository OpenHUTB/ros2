#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stanley 前轴横向偏差跟踪模块的统一入口。

示例::

    python3 main.py --mode demo                       # 全部场景 × 全部控制器，导出图表
    python3 main.py --mode demo --scenario circle     # 只跑圆弧场景
    python3 main.py --mode single --controller stanley --scenario sine
    python3 main.py --mode sweep --parameter k --values 0.6,1.0,1.6,2.4,3.2
    python3 main.py --mode ros                        # 启动 ROS 2 节点（需要 ROS 2 环境）

约定：入口文件名固定为 ``main.py``；不带参数时等价于 ``--mode demo``。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

# 允许直接 `python3 main.py` 运行：把本文件所在目录加入模块搜索路径
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np  # noqa: E402

from stanley_tracking import benchmark  # noqa: E402
from stanley_tracking.controllers import build_controller  # noqa: E402
from stanley_tracking.reference import SCENARIOS, build_scenario, from_csv  # noqa: E402
from stanley_tracking.simulator import Simulator, evaluate  # noqa: E402

DEFAULT_OUTPUT = os.path.join(_HERE, "results")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stanley 前轴横向偏差跟踪：离线仿真、参数扫描与效果图导出",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--mode", default="demo",
                        choices=["demo", "single", "sweep", "compare", "replay", "ros"],
                        help="运行模式：demo=场景矩阵，single=单次，sweep=参数扫描，"
                             "compare=后轴/前轴控制点对比，replay=回放 CSV 航点，ros=启动 ROS 2 节点")
    parser.add_argument("--scenario", default="all",
                        help=f"场景名，可选 {'/'.join(SCENARIOS)} 或 all")
    parser.add_argument("--controller", default="all",
                        help="控制器名（stanley/pure_pursuit/lookahead_pid）或 all")
    parser.add_argument("--speed", type=float, default=5.0, help="巡航速度 m/s")
    parser.add_argument("--wind", type=float, default=0.0, help="常值侧向风扰 m/s")
    parser.add_argument("--dt", type=float, default=0.02, help="仿真步长 s")
    parser.add_argument("--parameter", default="k", help="sweep 模式扫描的参数名")
    parser.add_argument("--values", default="0.6,1.0,1.6,2.4,3.2", help="sweep 模式的取值列表")
    parser.add_argument("--waypoints", default="", help="replay 模式的航点 CSV 路径")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="结果输出目录")
    parser.add_argument("--no-figures", action="store_true", help="只输出数值指标，不生成图表")
    parser.add_argument("--no-animation", action="store_true", help="不生成 GIF 动图")
    return parser.parse_args(argv)


def resolve_scenarios(name: str):
    if name == "all":
        return list(SCENARIOS)
    if name not in SCENARIOS:
        raise SystemExit(f"未知场景 '{name}'，可选: {', '.join(SCENARIOS)} 或 all")
    return [name]


def resolve_controllers(name: str):
    controllers = benchmark.default_controllers()
    if name == "all":
        return controllers
    if name not in controllers:
        raise SystemExit(f"未知控制器 '{name}'，可选: {', '.join(controllers)} 或 all")
    return {name: controllers[name]}


def build_path(args):
    if args.mode == "replay":
        if not args.waypoints:
            raise SystemExit("replay 模式必须提供 --waypoints 指向航点 CSV")
        return from_csv(args.waypoints)
    return build_scenario(args.scenario)


def run_demo(args) -> int:
    scenarios = resolve_scenarios(args.scenario)
    controllers = resolve_controllers(args.controller)
    config = benchmark.make_config(speed=args.speed, lateral_wind=args.wind, dt=args.dt)

    print(f"控制器: {', '.join(controllers)}")
    print(f"场景  : {', '.join(scenarios)}")
    print(f"速度  : {args.speed} m/s，侧向风扰 {args.wind} m/s，步长 {args.dt} s\n")

    traces, metrics = benchmark.run_matrix(controllers, scenarios, config)
    print(benchmark.format_table(metrics))

    os.makedirs(args.output, exist_ok=True)
    benchmark.metrics_to_csv(metrics, os.path.join(args.output, "metrics.csv"))
    benchmark.metrics_to_json(metrics, os.path.join(args.output, "metrics.json"))
    print(f"\n指标已导出: {os.path.join(args.output, 'metrics.csv')}")

    # 单场景下把所有控制器画在同一张图上；多场景时按场景分别出图
    if not args.no_figures:
        try:
            from stanley_tracking import visualization
        except Exception as exc:  # pragma: no cover
            print(f"跳过绘图：{exc}")
            return 0
        figures = []
        try:
            for scenario in scenarios:
                subset = [t for t in traces if t.scenario == scenario]
                path = build_scenario(scenario)
                figures.append(visualization.plot_trajectories(
                    subset, path, os.path.join(args.output, f"trajectory_{scenario}.png")))
                figures.append(visualization.plot_errors(
                    subset, os.path.join(args.output, f"error_{scenario}.png")))
            figures.append(visualization.plot_metrics(
                metrics, os.path.join(args.output, "metrics.png")))
            if not args.no_animation:
                best = [t for t in traces if t.controller == "stanley" and t.scenario == scenarios[0]]
                if best:
                    figures.append(visualization.animate_tracking(
                        best[0], build_scenario(scenarios[0]),
                        os.path.join(args.output, "tracking.gif")))
        except RuntimeError as exc:
            print(f"绘图失败：{exc}")
            return 0
        for figure in figures:
            print(f"已生成图表: {figure}")

    # 导出 Stanley 在第一个场景的完整时序，便于用 Excel 复现
    first = [t for t in traces if t.controller == "stanley" and t.scenario == scenarios[0]]
    if first:
        trace_csv = benchmark.trace_to_csv(first[0], os.path.join(args.output, f"trace_{scenarios[0]}.csv"))
        print(f"轨迹时序已导出: {trace_csv}")
    return 0


def run_single(args) -> int:
    controllers = resolve_controllers(args.controller)
    if len(controllers) != 1:
        raise SystemExit("single 模式请用 --controller 指定单个控制器")
    name, controller = next(iter(controllers.items()))
    path = build_path(args)
    scenario = args.scenario if args.mode == "demo" else "replay"
    config = benchmark.make_config(speed=args.speed, lateral_wind=args.wind, dt=args.dt)
    trace = Simulator(path, config).run(controller, scenario=scenario)
    metric = evaluate(trace)
    print(benchmark.format_table([metric]))
    os.makedirs(args.output, exist_ok=True)
    benchmark.trace_to_csv(trace, os.path.join(args.output, f"trace_{name}_{scenario}.csv"))
    if not args.no_figures:
        try:
            from stanley_tracking import visualization
            visualization.plot_trajectories([trace], path,
                                            os.path.join(args.output, f"trajectory_{name}_{scenario}.png"))
            visualization.plot_errors([trace], os.path.join(args.output, f"error_{name}_{scenario}.png"))
            visualization.animate_tracking(trace, path,
                                           os.path.join(args.output, f"tracking_{name}_{scenario}.gif"))
        except RuntimeError as exc:
            print(f"绘图失败：{exc}")
    return 0


def run_sweep(args) -> int:
    values = [float(v) for v in args.values.split(",") if v.strip()]
    scenarios = resolve_scenarios(args.scenario)
    print(f"扫描参数 {args.parameter}，取值 {values}，场景 {', '.join(scenarios)}\n")
    metrics = []
    for scenario in scenarios:
        metrics.extend(benchmark.run_sweep(scenario, args.parameter, values,
                                           controller_name="stanley"))
    print(benchmark.format_table(metrics))
    os.makedirs(args.output, exist_ok=True)
    benchmark.metrics_to_csv(metrics, os.path.join(args.output, f"sweep_{args.parameter}.csv"))
    if not args.no_figures:
        try:
            from stanley_tracking import visualization
            visualization.plot_metrics(metrics, os.path.join(args.output, f"sweep_{args.parameter}.png"),
                                       title_zh=f"参数扫描：{args.parameter}")
        except RuntimeError as exc:
            print(f"绘图失败：{exc}")
    return 0


def run_compare(args) -> int:
    """对比两种控制点（后轴 / 前轴）在各场景下的稳态偏差。

    这是本模块的主要结论之一：前轴控制点会超前一个轴距取值，
    在弯道上留下明显的稳态偏差；后轴控制点与运动学模型的原点一致。
    """
    scenarios = resolve_scenarios(args.scenario)
    metrics = []
    from stanley_tracking.controllers import StanleyController
    for scenario in scenarios:
        path = build_scenario(scenario)
        for point, label in (("rear_axle", "stanley_rear_axle"),
                             ("front_axle", "stanley_front_axle")):
            controller = StanleyController(control_point=point)
            config = benchmark.make_config(speed=args.speed, lateral_wind=args.wind, dt=args.dt)
            trace = Simulator(path, config).run(controller, scenario=scenario)
            metric = evaluate(trace)
            metric.controller = label
            metrics.append(metric)

    print(benchmark.format_table(metrics))
    os.makedirs(args.output, exist_ok=True)
    benchmark.metrics_to_csv(metrics, os.path.join(args.output, "control_point_compare.csv"))
    benchmark.metrics_to_json(metrics, os.path.join(args.output, "control_point_compare.json"))
    if not args.no_figures:
        try:
            from stanley_tracking import visualization
            visualization.plot_metrics(
                metrics, os.path.join(args.output, "control_point_compare.png"),
                title_zh="控制点对比：后轴 vs 前轴", title_en="Control point: rear vs front axle")
        except RuntimeError as exc:
            print(f"绘图失败：{exc}")
    print(f"\n对比结果已导出: {os.path.join(args.output, 'control_point_compare.csv')}")
    return 0


def run_ros(args) -> int:
    try:
        from stanley_tracking import stanley_node
    except ImportError as exc:
        raise SystemExit(f"无法导入 ROS 2 依赖（{exc}）。请先 source /opt/ros/<distro>/setup.bash") from exc
    stanley_node.main()
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.mode == "demo":
        return run_demo(args)
    if args.mode in ("single", "replay"):
        return run_single(args)
    if args.mode == "sweep":
        return run_sweep(args)
    if args.mode == "compare":
        return run_compare(args)
    if args.mode == "ros":
        return run_ros(args)
    raise SystemExit(f"未知模式 {args.mode}")


if __name__ == "__main__":
    sys.exit(main())
