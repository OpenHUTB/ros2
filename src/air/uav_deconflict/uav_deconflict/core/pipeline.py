#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端流水线：场景 → 冲突检测 → 解脱 → MuJoCo 闭环复飞 → 指标。"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from .conflict import detect
from .metrics import summarize
from .plan import Plan
from .resolve import Resolution, resolve_fleet
from .scenarios import Scenario, get_scenario
from .sim import SimResult, run_fleet


@dataclass
class CaseResult:
    scenario: Scenario
    baseline: List[Plan]
    resolution: Resolution
    summary: dict
    sim_before: Optional[SimResult] = None
    sim_after: Optional[SimResult] = None

    @property
    def name(self) -> str:
        return self.scenario.name


def run_case(name: str, sim: bool = True, progress: bool = False) -> CaseResult:
    """跑一个场景的完整流程。"""
    sc = get_scenario(name)
    baseline = [p.copy_with() for p in sc.plans]
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)

    sim_before = sim_after = None
    if sim:
        if progress:
            print("[%s] 基线航迹闭环复飞 ..." % name)
        sim_before = run_fleet(baseline, progress=progress)
        if progress:
            print("[%s] 解脱后航迹闭环复飞 ..." % name)
        sim_after = run_fleet([res.plans[p.uid] for p in baseline], progress=progress)

    summary = summarize(name, baseline, [res.plans[p.uid] for p in baseline],
                        sim_before, sim_after, sc.sep)
    return CaseResult(sc, baseline, res, summary, sim_before, sim_after)


def run_all(names: Sequence[str], sim: bool = True,
            progress: bool = False) -> List[CaseResult]:
    return [run_case(n, sim=sim, progress=progress) for n in names]


def to_json(results: Sequence[CaseResult], path: str) -> str:
    payload = []
    for r in results:
        payload.append(dict(
            scenario=r.name,
            title=r.scenario.title,
            description=r.scenario.description,
            plans=[p.summary() for p in r.baseline],
            actions=[dict(kind=a.kind, uid=a.uid, value=a.value,
                          cost=a.cost, note=a.note) for a in r.resolution.actions],
            summary=r.summary,
        ))
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def format_report(result: CaseResult) -> str:
    """生成控制台报告（也用于文档里的结果粘贴）。"""
    s = result.summary
    lines = []
    lines.append("=" * 72)
    lines.append("%s  —  %s" % (result.name, result.scenario.title))
    lines.append(result.scenario.description)
    lines.append("-" * 72)
    lines.append("间隔标准：水平 %.0f m / 垂直 %.0f m"
                 % (s["sep"]["horizontal"], s["sep"]["vertical"]))
    lines.append("解脱前：冲突 %d 段（%d 对机），最小水平 %.1f m，累计侵入 %.0f s"
                 % (s["before"]["conflicts"], s["before"]["conflict_pairs"],
                    s["before"]["min_h"], s["before"]["total_duration"]))
    lines.append("解脱后：冲突 %d 段，最小间隔裕度比见 JSON"
                 % s["after"]["conflicts"])
    lines.append("解脱动作：%d 个，总延误 %.1f s%s，最大单机延误 %.1f s"
                 % (len(result.resolution.actions), s["delay_total"],
                    ("（另有提前 %.1f s）" % s["advance_total"]
                     if s.get("advance_total") else ""),
                    s["delay_max"]))
    if result.resolution.actions:
        for a in result.resolution.actions:
            lines.append("  · %s（等效代价 %.1f s）" % (a.describe(), a.cost))
    lines.append("逐机结果：")
    lines.append("  %-8s %-20s %6s %6s %8s %8s" %
                 ("机号", "任务", "起飞机", "巡航高", "延误s", "航程m"))
    from .resolve import report_rows
    for row in report_rows(result.resolution, {p.uid: p for p in result.baseline}):
        lines.append("  %-8s %-20s %6.0f %6.0f %8.1f %8.0f" %
                     (row["uid"], row["label"][:18], row["t0_new"],
                      row["alt_new"], row["delay"], row["path_xy"]))
    for key, title in (("truth_before", "基线真飞"), ("truth_after", "解脱后真飞")):
        if key not in s:
            continue
        t = s[key]
        lines.append("%s：最小水平间隔 %.2f m（%s），侵入采样 %d 点"
                     % (title, t["min_h"], "↔".join(t["pair"]) or "-", t["violations"]))
        if "arrival_error_max" in t:
            lines.append("           最大到达偏差 %.2f m，最大跟踪误差 %.2f m"
                         % (t["arrival_error_max"], t["track_error_max"]))
    return "\n".join(lines)
