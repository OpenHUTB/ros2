#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验编排：批量跑场景 × 控制器，汇总指标并导出 CSV/JSON。"""

from __future__ import annotations

import csv
import json
import os
from dataclasses import asdict
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

from .controllers import BaseController, build_controller
from .reference import build_scenario, from_csv
from .simulator import Metrics, SimulationConfig, SimulationTrace, Simulator, SpeedProfile, evaluate


def default_controllers() -> Dict[str, BaseController]:
    """默认对比的控制器集合：Stanley 及其两种常见基线。"""
    return {
        "stanley": build_controller("stanley"),
        "pure_pursuit": build_controller("pure_pursuit"),
        "lookahead_pid": build_controller("lookahead_pid"),
    }


def make_config(speed: float = 5.0, lateral_wind: float = 0.0,
                dt: float = 0.02, initial_state: Sequence[float] | None = None) -> SimulationConfig:
    """构造仿真配置。"""
    return SimulationConfig(
        dt=dt,
        initial_state=None if initial_state is None else np.asarray(initial_state, dtype=float),
        lateral_wind=lateral_wind,
        speed_profile=SpeedProfile(target_speed=speed),
    )


def run_single(controller: BaseController, scenario: str, config: SimulationConfig | None = None,
               path=None) -> SimulationTrace:
    """跑单个"控制器 × 场景"组合。"""
    reference = path if path is not None else build_scenario(scenario)
    simulator = Simulator(reference, config or make_config())
    return simulator.run(controller, scenario=scenario)


def run_matrix(controllers: Dict[str, BaseController], scenarios: Iterable[str],
               config: SimulationConfig | None = None) -> Tuple[List[SimulationTrace], List[Metrics]]:
    """跑完整的"控制器 × 场景"矩阵，返回轨迹与指标。"""
    traces: List[SimulationTrace] = []
    metrics: List[Metrics] = []
    for scenario in scenarios:
        reference = build_scenario(scenario)
        for name, controller in controllers.items():
            sim_config = config or make_config()
            simulator = Simulator(reference, sim_config)
            trace = simulator.run(controller, scenario=scenario)
            traces.append(trace)
            metrics.append(evaluate(trace))
    return traces, metrics


def run_sweep(scenario: str, parameter: str, values: Sequence[float],
              controller_name: str = "stanley",
              base_config: SimulationConfig | None = None) -> List[Metrics]:
    """单参数扫描：观察横向增益 k、软化项 k_soft、速度等对跟踪性能的影响。"""
    results: List[Metrics] = []
    allowed = {"k", "k_soft", "feedforward_gain", "target_speed"}
    if parameter not in allowed:
        raise KeyError(f"可扫描参数: {', '.join(sorted(allowed))}")
    for value in values:
        kwargs = {}
        config = base_config or make_config()
        if parameter == "target_speed":
            config = make_config(speed=float(value),
                                 lateral_wind=config.lateral_wind,
                                 dt=config.dt,
                                 initial_state=config.initial_state)
        else:
            kwargs[parameter] = float(value)
        controller = build_controller(controller_name, **kwargs)
        reference = build_scenario(scenario)
        simulator = Simulator(reference, config)
        trace = simulator.run(controller, scenario=f"{scenario}@{parameter}={value:g}")
        results.append(evaluate(trace))
    return results


def metrics_to_csv(metrics: Sequence[Metrics], output: str) -> str:
    """把指标写为 CSV。"""
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    rows = [m.as_row() for m in metrics]
    if not rows:
        return output
    with open(output, "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return output


def metrics_to_json(metrics: Sequence[Metrics], output: str) -> str:
    """把指标写为 JSON。"""
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", encoding="utf-8") as handle:
        json.dump([m.as_row() for m in metrics], handle, ensure_ascii=False, indent=2)
    return output


def format_table(metrics: Sequence[Metrics]) -> str:
    """把指标渲染成等宽表格，便于终端查看与粘贴到文档。"""
    header = ["controller", "scenario", "mean|e|", "rms", "max|e|", "steady|e|",
              "head|psi|", "rate_rms", "conv_t"]
    lines = ["  ".join(f"{h:>12}" for h in header)]
    lines.append("-" * len(lines[0]))
    for m in metrics:
        row = [m.controller, m.scenario,
               f"{m.cross_track_mean_abs:.4f}", f"{m.cross_track_rms:.4f}",
               f"{m.cross_track_max_abs:.4f}", f"{m.cross_track_steady_abs:.4f}",
               f"{m.heading_mean_abs_deg:.3f}", f"{m.steering_rate_rms_deg:.2f}",
               f"{m.convergence_time:.2f}"]
        lines.append("  ".join(f"{v:>12}" for v in row))
    return "\n".join(lines)


def trace_to_csv(trace: SimulationTrace, output: str) -> str:
    """导出单条轨迹的时序数据，便于用 Excel 复现图表。"""
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "yaw", "v", "delta", "omega", "e", "psi",
                         "ref_x", "ref_y"])
        for i in range(len(trace)):
            writer.writerow([f"{trace.time[i]:.4f}", f"{trace.x[i]:.4f}", f"{trace.y[i]:.4f}",
                             f"{trace.yaw[i]:.6f}", f"{trace.speed[i]:.4f}",
                             f"{trace.steering[i]:.6f}", f"{trace.yaw_rate[i]:.6f}",
                             f"{trace.cross_track_error[i]:.6f}",
                             f"{trace.heading_error[i]:.6f}",
                             f"{trace.reference_xy[i, 0]:.4f}", f"{trace.reference_xy[i, 1]:.4f}"])
    return output
