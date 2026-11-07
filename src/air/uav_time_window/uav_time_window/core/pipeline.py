#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端流水线：场景 → 可行性预检 → EDF 贪心 → 局部改进 → 4D 航迹
→ MuJoCo 闭环真飞（抽 2 架）→ 指标汇总。
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .assign import AllocateResult, allocate, report_rows
from .mission import Scenario, screen_tasks
from .sim import SimResult, run_fleet
from .track import build_tracks
from .window import UavSchedule, fleet_report, objective

#: 闭环真飞抽取的机数（领域要求抽 2 架）
SIM_UAVS = 2


def pick_sim_uids(result: AllocateResult, n: int = SIM_UAVS) -> List[str]:
    """挑闭环验证的机：优先任务数多的（覆盖面大），平手取机号序。"""
    ranked = sorted(result.assign,
                    key=lambda uid: (-len(result.assign[uid]), uid))
    return [uid for uid in ranked if result.assign[uid]][:n]


@dataclass
class CaseResult:
    scenario: Scenario
    allocation: AllocateResult
    greedy_report: dict
    final_report: dict
    screen: List[dict] = field(default_factory=list)
    sim: Optional[SimResult] = None
    sim_uids: List[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.scenario.name


def run_case(name: str, sim: bool = True, progress: bool = False) -> CaseResult:
    """跑一个场景的完整流程。"""
    from .scenarios import get_scenario
    sc = get_scenario(name)

    screen = screen_tasks(sc)
    feasible = [r["tid"] for r in screen if not r["infeasible"]]
    dropped = [r for r in screen if r["infeasible"]]

    alloc = allocate(sc, feasible=feasible, dropped=dropped)
    uavs = {u.uid: u for u in sc.uavs}
    tasks = {t.tid: t for t in sc.tasks}

    g_scheds = {uid: _reschedule(uavs[uid], tids, tasks, sc)
                for uid, tids in alloc.greedy_assign.items()}
    greedy_report = fleet_report(g_scheds, sc.config)
    final_report = fleet_report(alloc.schedules, sc.config)

    case = CaseResult(scenario=sc, allocation=alloc,
                      greedy_report=greedy_report, final_report=final_report,
                      screen=screen)

    if sim:
        sim_uids = pick_sim_uids(alloc)
        tracks = build_tracks({u: alloc.schedules[u] for u in sim_uids},
                              uavs, tasks, alt=sc.config.alt,
                              climb_speed=sc.config.climb_speed)
        if progress:
            print("[%s] MuJoCo 闭环真飞 %s ..." % (name, ",".join(sim_uids)))
        case.sim = run_fleet([tracks[u] for u in sim_uids], progress=progress)
        case.sim_uids = sim_uids
    return case


def _reschedule(uav, tids, tasks, sc) -> UavSchedule:
    from .window import schedule_uav
    return schedule_uav(uav, tids, tasks, sc.config)


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
            uavs=[dict(uid=u.uid, x=u.x, y=u.y, speed=u.speed,
                       endurance=u.endurance) for u in r.scenario.uavs],
            tasks=[dict(tid=t.tid, x=t.x, y=t.y, earliest=t.earliest,
                        latest=t.latest, service_time=t.service_time,
                        priority=t.priority) for t in r.scenario.tasks],
            screen=r.screen,
            allocation=r.allocation.as_dict(),
            greedy=r.greedy_report,
            final=r.final_report,
            per_uav=report_rows(r.allocation),
            sim=(dict(uids=r.sim_uids,
                      arrivals=r.sim.arrivals_as_dict(),
                      **r.sim.summary(),
                      max_track_error={k: round(v, 2)
                                       for k, v in r.sim.max_track_error.items()},
                      steps=r.sim.steps, wall_time=round(r.sim.wall_time, 1))
                 if r.sim else None),
        ))
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def format_report(result: CaseResult) -> str:
    """生成控制台报告（也用于文档里的结果粘贴）。"""
    sc, a = result.scenario, result.allocation
    g, f = result.greedy_report, result.final_report
    lines = []
    lines.append("=" * 72)
    lines.append("%s  —  %s" % (sc.name, sc.title))
    lines.append(sc.description)
    lines.append("-" * 72)
    lines.append("目标函数 = Σ正延误 + λ×(最大单机延误 − 最小单机延误)，λ=%.2f"
                 % sc.config.lambda_balance)
    lines.append("贪心（EDF）：总延误 %6.1f s  最大单机 %6.1f s  目标 %7.1f"
                 % (g["delay_total"], g["delay_max_uav"], g["objective"]))
    lines.append("改进后　　：总延误 %6.1f s  最大单机 %6.1f s  目标 %7.1f"
                 % (f["delay_total"], f["delay_max_uav"], f["objective"]))
    lines.append("改进动作：互换 %d 次、机间迁移 %d 次（%d 轮，严格下降接受）"
                 % (a.swaps, a.transfers, a.rounds))
    if a.dropped:
        lines.append("不可达任务 %d 个：%s"
                     % (len(a.dropped),
                        "；".join("%s(%s)" % (d["tid"], d["reason"])
                                  for d in a.dropped)))
    lines.append("逐机分配（改进后）：")
    lines.append("  %-8s %-34s %8s %8s %8s" %
                 ("机号", "任务序列", "延误s", "悬停s", "完工s"))
    for row in report_rows(a):
        lines.append("  %-8s %-34s %8.1f %8.1f %8.1f" %
                     (row["uid"], "→".join(row["tasks"]) or "—",
                      row["lateness_total"], row["wait_total"], row["makespan"]))
    if result.sim is not None:
        s = result.sim
        lines.append("闭环真飞（%s）：任务点 %d 个，实际−计划到达偏差均值 %s s、"
                     "最大 %s s，未检出 %s"
                     % (",".join(result.sim_uids),
                        s.summary()["n_arrivals"],
                        s.summary()["deviation_mean"],
                        s.summary()["deviation_max"],
                        s.summary()["missed"] or "无"))
        for rec in s.arrivals_as_dict():
            lines.append("  · %s %s 计划 %.1f s 实际 %s s 窗口 %s 状态 %s"
                         % (rec["uid"], rec["tid"], rec["planned"],
                            rec["actual"] if rec["actual"] is not None else "—",
                            rec["window"], rec["status"]))
    return "\n".join(lines)
