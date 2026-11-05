#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端流水线：场景 → 基线评估 → 降噪优化 → MuJoCo 闭环复飞 → 指标。

闭环部分回答 SPEC 的两个硬问题：

* 优化后航迹（爬升角 12° / 更高巡航）在 MuJoCo 里仍可控（跟踪误差 < 8 m）；
* 用**真实位置日志**重算敏感点噪声，与计划值偏差在合理范围（证明结论
  不是拿计划值凑数）。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from .exposure import (SensitivePoint, contour_summary, grid_footprint,
                       laeq_at_points, laeq_from_log, objective)
from .mitigate import laeq_table, mitigate_fleet
from .scenarios import OBS_PERIOD, Scenario, get_scenario
from .sim import SimResult, run_flight

#: 敏感点噪声重算时使用的航迹采样步长（m）
DS = 2.0
#: 足迹等值线级别（dB）
CONTOUR_LEVELS = (45.0, 55.0, 65.0)
#: 闭环验收：全程最大跟踪误差上限（m）
TRACK_ERROR_LIMIT = 8.0


@dataclass
class CaseResult:
    scenario: Scenario
    baseline: List[Flight]
    resolution: Resolution
    summary: dict
    sim_before: Optional[SimResult] = None
    sim_after: Optional[SimResult] = None

    @property
    def name(self) -> str:
        return self.scenario.name


def _pick_sim_flight(flights: Sequence[Flight],
                     points: Sequence[SensitivePoint]) -> str:
    """抽一架对敏感点噪声贡献最大的飞机做闭环复飞。"""
    best, best_obj = None, -math.inf
    for f in flights:
        obj = objective([f], points, DS)
        if obj > best_obj:
            best, best_obj = f.uid, obj
    return best


def _closed_loop_block(flight: Flight, sim: SimResult,
                       points: Sequence[SensitivePoint],
                       repeat: int = 1, t_obs: float = 3600.0) -> dict:
    """真飞指标 + 真实高度序列重算的敏感点 LAeq（对照计划值）。

    真实 LAeq 与计划值用同一套归一（repeat 次事件 / t_obs 观测期），
    其中真实事件能量取自 MuJoCo 记录的活跃时段位置日志。
    """
    plan_laeq = laeq_at_points([flight], points, DS, repeat=repeat,
                               t_obs=t_obs)
    mask = ((sim.times >= flight.start_time - 1e-9) &
            (sim.times <= flight.end_time + 1e-9))
    t_real = float(sim.times[mask][-1] - sim.times[mask][0])
    real_laeq = {}
    for pt in points:
        ev = laeq_from_log(sim.times[mask], sim.pos[mask], (pt.x, pt.y))
        # 事件能量 × repeat / t_obs，与计划口径一致
        real_laeq[pt.name] = round(
            ev + 10.0 * math.log10(repeat * t_real / t_obs), 2)
    return dict(
        sim=sim.to_dict(),
        track_error_limit=TRACK_ERROR_LIMIT,
        controllable=bool(sim.max_track_error < TRACK_ERROR_LIMIT),
        plan_laeq={k: round(v, 2) if math.isfinite(v) else None
                   for k, v in plan_laeq.items()},
        real_laeq=real_laeq,
        laeq_abs_dev={k: round(abs(real_laeq[k] - plan_laeq[k]), 2)
                      for k in real_laeq
                      if math.isfinite(plan_laeq[k])})


def run_case(name: str, sim: bool = True) -> CaseResult:
    """跑一个场景的完整流程。"""
    sc = get_scenario(name)
    baseline = [f.copy_with() for f in sc.flights]
    repeat, t_obs = sc.events_per_hour, OBS_PERIOD
    res = mitigate_fleet(baseline, sc.points, sc.params, ds=DS,
                         repeat=repeat, t_obs=t_obs)
    optimized = [res.plans[f.uid] for f in baseline]

    before_grid, extent = grid_footprint(baseline, ds=DS, repeat=repeat,
                                         t_obs=t_obs)
    after_grid, _ = grid_footprint(optimized, ds=DS, extent=extent,
                                   repeat=repeat, t_obs=t_obs)
    levels = CONTOUR_LEVELS

    summary: dict = dict(
        scenario=name,
        title=sc.title,
        description=sc.description,
        points=[p.to_dict() for p in sc.points],
        baseline_flights=[f.to_dict() for f in baseline],
        optimized_flights=[f.to_dict() for f in optimized],
        actions=[dict(kind=a.kind, uid=a.uid, value=round(a.value, 1),
                      note=a.note) for a in res.actions],
        objective_before=round(res.objective_before, 1),
        objective_after=round(res.objective_after, 1),
        improvement_db=round(res.improvement_db, 1),
        path_before_m=round(res.path_before, 1),
        path_after_m=round(res.path_after, 1),
        path_increase_pct=round(res.path_increase_ratio * 100, 2),
        max_path_increase_pct=round(res.max_path_increase * 100, 1),
        constraint_ok=bool(res.path_increase_ratio <=
                           res.max_path_increase + 1e-9),
        events_per_hour=repeat,
        obs_period_s=t_obs,
        laeq_table=laeq_table(baseline, optimized, sc.points, DS,
                              repeat=repeat, t_obs=t_obs),
        footprint=dict(
            extent=[round(v, 1) for v in extent],
            before=contour_summary(before_grid, extent, levels),
            after=contour_summary(after_grid, extent, levels)),
    )

    sim_before = sim_after = None
    if sim:
        uid = _pick_sim_flight(baseline, sc.points)
        fb = {f.uid: f for f in baseline}[uid]
        fa = {f.uid: f for f in optimized}[uid]
        print("[%s] 基线航迹闭环复飞（%s）..." % (name, uid))
        sim_before = run_flight(fb)
        print("[%s] 优化后航迹闭环复飞（%s，爬升角 %.0f°、巡航 %.0f m）..."
              % (name, uid, fa.climb_angle, fa.alt))
        sim_after = run_flight(fa)
        summary["closed_loop"] = dict(
            uid=uid,
            before=_closed_loop_block(fb, sim_before, sc.points, repeat, t_obs),
            after=_closed_loop_block(fa, sim_after, sc.points, repeat, t_obs))

    return CaseResult(sc, baseline, res, summary, sim_before, sim_after)


def run_all(names: Sequence[str], sim: bool = True) -> List[CaseResult]:
    return [run_case(n, sim=sim) for n in names]


def to_json(results: Sequence[CaseResult], path: str) -> str:
    payload = [r.summary for r in results]
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def format_report(result: CaseResult) -> str:
    """生成控制台报告（也用于文档里的结果粘贴）。"""
    s = result.summary
    lines = []
    lines.append("=" * 72)
    lines.append("%s  —  %s" % (result.name, s["title"]))
    lines.append(s["description"])
    lines.append("-" * 72)
    lines.append("降噪动作 %d 个（航程增幅 %.2f%%，约束 ≤ %.0f%%%s）："
                 % (len(s["actions"]), s["path_increase_pct"],
                    s["max_path_increase_pct"],
                    "，达标" if s["constraint_ok"] else "，超限！"))
    for a in s["actions"]:
        lines.append("  · %s %s %.0f" % (a["uid"], a["kind"], a["value"]))
    lines.append("加权目标：%.1f dB → %.1f dB（改善 %.1f dB）"
                 % (s["objective_before"], s["objective_after"],
                    s["improvement_db"]))
    lines.append("%-10s %-8s %10s %10s %8s" %
                 ("敏感点", "类型", "前 LAeq", "后 LAeq", "ΔdB"))
    for row in s["laeq_table"]:
        lines.append("%-10s %-8s %10s %10s %8s%s" %
                     (row["point"], row["kind"],
                      row["laeq_before"], row["laeq_after"], row["delta_db"],
                      "（超标→达标）" if row["before_exceed"] and
                      not row["after_exceed"] else ""))
    if "closed_loop" in s:
        cl = s["closed_loop"]
        for stage in ("before", "after"):
            c = cl[stage]
            lines.append("%s真飞（%s）：最大跟踪误差 %.2f m（限 %.0f m%s），"
                         "终点误差 %.2f m；真实高度重算 LAeq 与计划值最大偏差 %.1f dB"
                         % ("优化" if stage == "after" else "基线", cl["uid"],
                            c["sim"]["max_track_error"],
                            c["track_error_limit"],
                            "，可控" if c["controllable"] else "，超限！",
                            c["sim"]["final_error"],
                            max(c["laeq_abs_dev"].values())
                            if c["laeq_abs_dev"] else 0.0))
    return "\n".join(lines)
