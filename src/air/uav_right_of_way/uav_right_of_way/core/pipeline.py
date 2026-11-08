#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端流水线：场景 → 基线/规则启用闭环 → 指标 → JSON 与控制台报告。"""
from __future__ import annotations

import json
import os
from typing import List, Optional, Sequence

import numpy as np

from .detect import DetectParams
from .rules import RuleParams
from .scenarios import SCENARIOS, Scenario, get_scenario
from .sim import (CLEAR_HOLD_S, DECIDE_DT, RECOVERY_MIN_S, RunResult,
                  run_fleet)


def run_case(name: str, sim: bool = True, progress: bool = False,
             detect_params: Optional[DetectParams] = None,
             rule_params: Optional[RuleParams] = None) -> dict:
    """跑一个场景：基线（无规则）与规则启用各闭环一遍，返回 JSON 友好结果。"""
    sc = get_scenario(name)
    dp = detect_params or DetectParams()
    rp = rule_params or RuleParams()

    if progress:
        print("[%s] %s — 基线（无规则直飞）..." % (name, sc.title))
    baseline = run_fleet(sc.plans, enable_rules=False, progress=progress,
                         use_mujoco=sim, detect_params=dp, rule_params=rp)
    if progress:
        print("[%s] 规则启用（在线让路 + 余弦恢复）..." % name)
    rules = run_fleet(sc.plans, enable_rules=True, progress=progress,
                      use_mujoco=sim, detect_params=dp, rule_params=rp)

    return _build_result(sc, baseline, rules, dp, rp, sim)


def run_all(names: Sequence[str], sim: bool = True, progress: bool = False,
            detect_params: Optional[DetectParams] = None,
            rule_params: Optional[RuleParams] = None) -> List[dict]:
    return [run_case(n, sim=sim, progress=progress,
                     detect_params=detect_params, rule_params=rule_params)
            for n in names]


def _traj_payload(res: RunResult, stride: int = 10,
                  prec: int = 1) -> dict:
    """轨迹降采样落盘（每 stride 个记录点取 1 个，约 2 Hz）。"""
    out = {}
    for uid, arr in sorted(res.pos.items()):
        out[uid] = [[round(float(x), prec) for x in row]
                    for row in arr[::stride]]
    times = [round(float(t), 2) for t in res.times[::stride]]
    return dict(times=times, agents=out)


def _build_result(sc: Scenario, baseline: RunResult, rules: RunResult,
                  dp: DetectParams, rp: RuleParams, sim: bool) -> dict:
    return dict(
        scenario=sc.name,
        title=sc.title,
        description=sc.description,
        closed_loop=bool(sim),
        params=dict(
            horizon_s=dp.horizon_s,
            cpa_threshold_m=dp.cpa_threshold_m,
            turn_right_deg=rp.turn_right_deg,
            descend_m=rp.descend_m,
            slow_fraction=rp.slow_fraction,
            overtake_lateral_m=rp.overtake_lateral_m,
            decision_interval_s=DECIDE_DT,
            clear_hold_s=CLEAR_HOLD_S,
            recovery_min_s=RECOVERY_MIN_S,
        ),
        plans=[p.summary() for p in sc.plans],
        baseline=baseline.summary(dp),
        rules=dict(rules.summary(dp),
                   decision_log=[d.as_dict() for d in rules.decisions]),
        trajectories=dict(baseline=_traj_payload(baseline),
                          rules=_traj_payload(rules)),
        wall_time_s=round(baseline.wall_time + rules.wall_time, 1),
    )


def to_json(results: Sequence[dict], path: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    # 轨迹不落盘（占地方，评审也用不上）；出图用 tools/make_gif.py 重跑
    trimmed = []
    for r in results:
        r = dict(r)
        r.pop("trajectories", None)
        trimmed.append(r)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(trimmed, fh, ensure_ascii=False, indent=2)
    return path


def format_report(result: dict) -> str:
    """单场景控制台报告。"""
    lines = []
    lines.append("=" * 72)
    lines.append("%s  —  %s" % (result["scenario"], result["title"]))
    lines.append(result["description"])
    lines.append("-" * 72)
    lines.append("判定参数：预测 %.0f s 内 CPA < %.0f m 触发决策"
                 % (result["params"]["horizon_s"],
                    result["params"]["cpa_threshold_m"]))
    b = result["baseline"]
    r = result["rules"]
    lines.append("基线（无规则）：最小间距 %.2f m（%s），40 m 内采样 %d 点"
                 % (b["min_separation_m"], "↔".join(b["min_pair"]),
                    b["below_threshold_samples"]))
    lines.append("规则启用  ：最小间距 %.2f m（%s），决策 %d 次"
                 % (r["min_separation_m"], "↔".join(r["min_pair"]),
                    r["decisions"]))
    lines.append("平均航迹偏离：基线 %.2f m → 规则 %.2f m"
                 % (b["mean_deviation_m"], r["mean_deviation_m"]))
    if r.get("decision_log"):
        lines.append("决策日志（前 8 条）：")
        for d in r["decision_log"][:8]:
            lines.append("  t=%5.1fs  %s  %s  %s" %
                         (d["t"], d["uid"], d["action"]["kind"], d["reason"]))
    return "\n".join(lines)


def format_table(results: Sequence[dict]) -> str:
    """汇总表：基线 vs 规则启用的最小间距 / 决策次数 / 平均偏离。"""
    lines = []
    lines.append("=" * 84)
    lines.append("%-10s %14s %14s %8s %12s %12s" %
                 ("场景", "基线最小间距m", "规则最小间距m", "决策次数",
                  "基线偏离m", "规则偏离m"))
    for r in results:
        b, rl = r["baseline"], r["rules"]
        lines.append("%-10s %14.2f %14.2f %8d %12.2f %12.2f" %
                     (r["scenario"], b["min_separation_m"],
                      rl["min_separation_m"], rl["decisions"],
                      b["mean_deviation_m"], rl["mean_deviation_m"]))
    return "\n".join(lines)
