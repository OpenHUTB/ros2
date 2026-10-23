#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量实验编排：场景 × 控制器矩阵、指标导出、综合排名。

跨场景直接比较绝对数值会被"场景难度"带偏，因此除了原始指标，
还按场景内部对各指标排名，再汇总平均名次（越小越好）作为综合结论。
"""

from __future__ import annotations

import csv
import json
import os
from typing import Dict, List, Sequence, Tuple

from .controllers import BaseController, build_controller
from .geometry import SCENARIOS, build_scenario
from .simulation import (
    Metrics, SimulationConfig, SpeedProfile, evaluate, rank_controllers,
)

CONTROLLERS = ("pure_pursuit", "stanley", "lqr")


def default_config(speed: float = 8.0, model: str = "kinematic",
                   dt: float = 0.02) -> SimulationConfig:
    return SimulationConfig(dt=dt, model=model, speed=SpeedProfile(cruise_speed=speed))


def run_matrix(controllers: Sequence[str] = CONTROLLERS,
               scenarios: Sequence[str] = tuple(SCENARIOS),
               config: SimulationConfig | None = None,
               overrides: Dict[str, dict] | None = None) -> Tuple[List, List[Metrics]]:
    """跑完整的"控制器 × 场景"矩阵。

    ``overrides`` 可为单个控制器传入额外构造参数，例如
    ``{"stanley": {"k": 2.0}}``，便于做参数敏感性分析。
    """
    from .simulation import Simulator

    traces, metrics = [], []
    overrides = overrides or {}
    for scenario in scenarios:
        path = build_scenario(scenario)
        for name in controllers:
            cfg = config or default_config()
            ctrl: BaseController = build_controller(name, **overrides.get(name, {}))
            sim = Simulator(path, cfg)
            trace = sim.run(ctrl, scenario=scenario)
            traces.append(trace)
            metrics.append(evaluate(trace))
    return traces, metrics


def format_table(metrics: Sequence[Metrics]) -> str:
    """渲染成等宽表格，便于在终端核对与粘贴到文档。"""
    header = ["controller", "scenario", "mean|e|", "rms", "max|e|", "steady|e|",
              "|psi|", "rate_rms", "conv_t", "off%"]
    lines = ["  ".join(f"{h:>12}" for h in header), "-" * 118]
    for m in metrics:
        row = [m.controller, m.scenario,
               f"{m.e_mean_abs:.4f}", f"{m.e_rms:.4f}", f"{m.e_max_abs:.4f}",
               f"{m.e_steady_abs:.4f}", f"{m.psi_mean_abs_deg:.2f}",
               f"{m.delta_rate_rms_deg:.2f}", f"{m.convergence_time:.2f}",
               f"{100 * m.off_track_ratio:.1f}"]
        lines.append("  ".join(f"{v:>12}" for v in row))
    return "\n".join(lines)


def format_ranking(metrics: Sequence[Metrics]) -> str:
    """把综合排名渲染成表格。"""
    ranks = rank_controllers(metrics)
    lines = [f"{'controller':<16}{'平均名次':>10}{'指标名次和':>12}{'参与比较次数':>14}",
             "-" * 54]
    for name, entry in sorted(ranks.items(), key=lambda kv: kv[1]["rank_mean"]):
        lines.append(f"{name:<16}{entry['rank_mean']:>10.2f}"
                     f"{entry['rank_sum']:>12.0f}{entry['rank_count']:>14.0f}")
    return "\n".join(lines)


def metrics_to_csv(metrics: Sequence[Metrics], output: str) -> str:
    """导出指标表。

    编码用 **utf-8 无 BOM**：所有列名均为 ASCII，BOM 没有存在的必要，
    而部分 Excel 版本会把 BOM 当成首列表头的一部分显示成乱码。
    """
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    rows = [m.as_dict() for m in metrics]
    if not rows:
        return output
    with open(output, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return output


def metrics_to_json(metrics: Sequence[Metrics], output: str,
                    extra: Dict | None = None) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    payload = {"metrics": [m.as_dict() for m in metrics],
               "ranking": rank_controllers(metrics)}
    if extra:
        payload.update(extra)
    with open(output, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return output


def trace_to_csv(trace, output: str) -> str:
    """导出单条轨迹时序，便于用 Excel / pandas 复现曲线（utf-8 无 BOM）。"""
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "yaw", "v", "delta", "e", "psi"])
        for i in range(len(trace)):
            writer.writerow([f"{trace.t[i]:.4f}", f"{trace.x[i]:.4f}", f"{trace.y[i]:.4f}",
                             f"{trace.yaw[i]:.6f}", f"{trace.v[i]:.4f}",
                             f"{trace.delta[i]:.6f}", f"{trace.e[i]:.6f}",
                             f"{trace.psi[i]:.6f}"])
    return output


def summarize(metrics: Sequence[Metrics]) -> str:
    """给出一段文字结论，便于直接放进 PR 描述。"""
    ranks = rank_controllers(metrics)
    order = [n for n, _ in sorted(ranks.items(), key=lambda kv: kv[1]["rank_mean"])]
    lines = []
    for name in order:
        subset = [m for m in metrics if m.controller == name]
        steady = sum(m.e_steady_abs for m in subset) / len(subset)
        rms = sum(m.e_rms for m in subset) / len(subset)
        conv = sum(m.convergence_time for m in subset) / len(subset)
        lines.append(f"  {name:<14} 平均稳态偏差 {steady:.4f} m | 平均 RMS {rms:.4f} m | "
                     f"平均收敛 {conv:.2f} s | 平均名次 {ranks[name]['rank_mean']:.2f}")
    return "\n".join(lines)
