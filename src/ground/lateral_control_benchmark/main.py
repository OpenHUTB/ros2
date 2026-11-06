#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""横向控制算法对比实验台 —— 统一入口。

示例::

    python3 main.py --mode bench                       # 四场景 × 三控制器
    python3 main.py --mode bench --scenario arc --model dynamic
    python3 main.py --mode bench --speed 15            # 高速工况
    python3 main.py --mode single --controller stanley --scenario sine
    python3 main.py --mode sweep --param stanley.k --values 1.0,2.0,4.0
    python3 main.py --mode ros                         # 启动 ROS 1 节点

约定：入口文件名固定为 ``main.py``；不带参数时等价于 ``--mode bench``。
"""

from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np  # noqa: E402

from lateral_control_benchmark import benchmark  # noqa: E402
from lateral_control_benchmark.controllers import build_controller  # noqa: E402
from lateral_control_benchmark.geometry import SCENARIOS, build_scenario  # noqa: E402
from lateral_control_benchmark.simulation import (  # noqa: E402
    Simulator, SpeedProfile, evaluate,
)

DEFAULT_OUTPUT = os.path.join(_HERE, "results")


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="纯跟踪 / Stanley / LQR 横向控制算法定量对比（纯 Python，不依赖 ROS 与仿真器）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--mode", default="bench",
                   choices=["bench", "single", "sweep", "ros"],
                   help="bench=场景×控制器矩阵；single=单次；sweep=参数扫描；ros=启动 ROS 节点")
    p.add_argument("--scenario", default="all",
                   help=f"场景名（{'/'.join(SCENARIOS)}）或 all")
    p.add_argument("--controller", default="all",
                   help="控制器名（pure_pursuit/stanley/lqr）或 all")
    p.add_argument("--speed", type=float, default=8.0, help="巡航速度 m/s")
    p.add_argument("--model", default="kinematic", choices=["kinematic", "dynamic"],
                   help="车辆模型：kinematic=运动学自行车，dynamic=二自由度动力学")
    p.add_argument("--dt", type=float, default=0.02, help="仿真步长 s")
    p.add_argument("--duration", type=float, default=0.0,
                   help="单次仿真的最长时间 s（0 表示用默认值）")
    p.add_argument("--param", default="stanley.k",
                   help="sweep 模式要扫描的参数，形如 stanley.k / lqr.ki / pure_pursuit.lookahead_gain")
    p.add_argument("--values", default="1.0,2.0,4.0", help="sweep 模式的取值列表")
    p.add_argument("--output", default=DEFAULT_OUTPUT, help="结果输出目录")
    p.add_argument("--no-figures", action="store_true", help="只输出数值，不生成图表")
    return p.parse_args(argv)


def resolve_scenarios(name: str):
    if name == "all":
        return list(SCENARIOS)
    if name not in SCENARIOS:
        raise SystemExit(f"未知场景 '{name}'，可选: {', '.join(SCENARIOS)} 或 all")
    return [name]


def run_bench(args) -> int:
    scenarios = resolve_scenarios(args.scenario)
    controllers = list(benchmark.CONTROLLERS) if args.controller == "all" else [args.controller]
    config = benchmark.default_config(speed=args.speed, model=args.model, dt=args.dt)

    print(f"车辆模型: {args.model} | 巡航速度: {args.speed} m/s | 步长: {args.dt} s")
    print(f"场景: {', '.join(scenarios)}")
    print(f"控制器: {', '.join(controllers)}\n")

    traces, metrics = benchmark.run_matrix(controllers, scenarios, config)
    print(benchmark.format_table(metrics))
    print("\n综合排名（按场景内部各指标名次汇总，越小越好）")
    print(benchmark.format_ranking(metrics))
    print("\n汇总")
    print(benchmark.summarize(metrics))

    os.makedirs(args.output, exist_ok=True)
    benchmark.metrics_to_csv(metrics, os.path.join(args.output, "metrics.csv"))
    benchmark.metrics_to_json(metrics, os.path.join(args.output, "metrics.json"),
                              extra={"model": args.model, "speed": args.speed})
    print(f"\n指标已导出: {os.path.join(args.output, 'metrics.csv')}")

    if not args.no_figures:
        try:
            from lateral_control_benchmark import viz
            figures = []
            for scenario in scenarios:
                subset = [t for t in traces if t.scenario == scenario]
                figures.append(viz.plot_trajectories(
                    subset, build_scenario(scenario),
                    os.path.join(args.output, f"trajectory_{scenario}.png")))
                figures.append(viz.plot_errors(
                    subset, os.path.join(args.output, f"error_{scenario}.png")))
            figures.append(viz.plot_metrics(metrics, os.path.join(args.output, "metrics.png")))
            for f in figures:
                print(f"已生成图表: {f}")
        except Exception as exc:
            print(f"跳过绘图：{exc}")

    first = [t for t in traces if t.controller == "lqr" and t.scenario == scenarios[0]]
    if first:
        out = benchmark.trace_to_csv(first[0], os.path.join(args.output,
                                                            f"trace_lqr_{scenarios[0]}.csv"))
        print(f"轨迹时序已导出: {out}")
    return 0


def run_single(args) -> int:
    if args.controller == "all":
        raise SystemExit("single 模式需要用 --controller 指定一个控制器")
    scenario = resolve_scenarios(args.scenario)[0]
    cfg = benchmark.default_config(speed=args.speed, model=args.model, dt=args.dt)
    if args.duration > 0:
        cfg.duration = args.duration
    trace = Simulator(build_scenario(scenario), cfg).run(build_controller(args.controller), scenario)
    m = evaluate(trace)
    print(benchmark.format_table([m]))
    os.makedirs(args.output, exist_ok=True)
    print(f"轨迹已导出: {benchmark.trace_to_csv(trace, os.path.join(args.output, f'trace_{args.controller}_{scenario}.csv'))}")
    return 0


def run_sweep(args) -> int:
    if "." not in args.param:
        raise SystemExit("--param 需形如 stanley.k")
    ctrl_name, key = args.param.split(".", 1)
    values = [float(v) for v in args.values.split(",") if v.strip()]
    scenarios = resolve_scenarios(args.scenario)
    print(f"扫描 {ctrl_name}.{key} = {values}\n")
    metrics = []
    for value in values:
        traces, batch = benchmark.run_matrix(
            [ctrl_name], scenarios, benchmark.default_config(speed=args.speed, model=args.model),
            overrides={ctrl_name: {key: value}})
        for m in batch:
            m.controller = f"{ctrl_name}[{key}={value:g}]"
        metrics.extend(batch)
    print(benchmark.format_table(metrics))
    os.makedirs(args.output, exist_ok=True)
    out = benchmark.metrics_to_csv(metrics, os.path.join(args.output, f"sweep_{ctrl_name}_{key}.csv"))
    print(f"\n扫描结果已导出: {out}")
    return 0


def run_ros(args) -> int:
    try:
        from lateral_control_benchmark import ros_node
    except ImportError as exc:
        raise SystemExit(f"无法导入 ROS 依赖（{exc}）。请先 source /opt/ros/noetic/setup.bash") from exc
    ros_node.main()
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.mode == "bench":
        return run_bench(args)
    if args.mode == "single":
        return run_single(args)
    if args.mode == "sweep":
        return run_sweep(args)
    if args.mode == "ros":
        return run_ros(args)
    raise SystemExit(f"未知模式 {args.mode}")


if __name__ == "__main__":
    sys.exit(main())
