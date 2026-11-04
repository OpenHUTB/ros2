#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端流水线：场景 → 排队前/后调度 → 指标对比 → MuJoCo 闭环抽飞。"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .capacity import capacity_scan, saturation_point
from .queue import (ARRIVAL, DEPARTURE, Assignment, QueueConfig, Request,
                    assign_pads_no_wait, find_conflicts, schedule,
                    schedule_metrics)
from .scenarios import (LAYOUT, Scenario, get_scenario)
from .track import Track, arrival_track, departure_track

#: 闭环抽飞的架次数（题目要求 2~3 架）
SIM_PICKS = 3


@dataclass
class CaseResult:
    scenario: Scenario
    before: object                    # Schedule（排队前 = 不排队直飞）
    after: object                     # Schedule（排队后）
    metrics_before: dict
    metrics_after: dict
    sim: Optional[dict] = None

    @property
    def name(self) -> str:
        return self.scenario.name


# --------------------------------------------------------------------------- #
# 算法层
# --------------------------------------------------------------------------- #
def run_case(name: str, sim: bool = False, progress: bool = False) -> CaseResult:
    """跑一个场景：排队前 vs 排队后，可选闭环抽飞。"""
    sc = get_scenario(name)
    cfg = sc.config
    before = assign_pads_no_wait(sc.requests, cfg)
    after = schedule(sc.requests, cfg, sc.strategy)
    mb = schedule_metrics(before.assignments, cfg, len(sc.requests))
    ma = schedule_metrics(after.assignments, cfg, len(sc.requests))
    sim_out = run_case_sim(sc, before, after, progress) if sim else None
    return CaseResult(sc, before, after, mb, ma, sim_out)


def run_all(names: Sequence[str], sim: bool = False,
            progress: bool = False) -> List[CaseResult]:
    return [run_case(n, sim=sim, progress=progress) for n in names]


def run_capacity(cfg: Optional[QueueConfig] = None) -> dict:
    scan = capacity_scan(cfg=cfg)
    return dict(scan=scan, saturation=saturation_point(scan))


# --------------------------------------------------------------------------- #
# MuJoCo 闭环抽飞
# --------------------------------------------------------------------------- #
def select_sim_ops(requests: Sequence[Request]) -> List[Request]:
    """挑 3 个架次闭环：优先含高优先级（急救），取请求时刻最接近的
    两个离场，再配一个时刻相近的进场——离场对用来验证离场间隔，
    进场用来验证接地执行，窗口尽量短。"""
    deps = sorted((r for r in requests if r.kind == DEPARTURE),
                  key=lambda r: r.t_request)
    arrs = sorted((r for r in requests if r.kind == ARRIVAL),
                  key=lambda r: r.t_request)
    if not deps:
        return arrs[:SIM_PICKS]
    # 基准架次：优先级最高者（平手取最早）
    anchor = min(deps, key=lambda r: (-r.priority, r.t_request))
    others = [r for r in deps if r is not anchor]
    others.sort(key=lambda r: abs(r.t_request - anchor.t_request))
    picked = [anchor] + others[:SIM_PICKS - 2]
    if arrs and len(picked) < SIM_PICKS:
        t_ref = picked[-1].t_request
        picked.append(min(arrs, key=lambda r: abs(r.t_request - t_ref)))
    picked.sort(key=lambda r: r.t_request)
    return picked[:SIM_PICKS]


def _pick_track(req: Request, t_run: float, pad: str) -> Track:
    # 仿真机队以 uid 为键：同一架 UAV 的离场/进场航迹加后缀避免覆盖
    sim_uid = "%s#%s" % (req.uid, req.kind)
    if req.kind == DEPARTURE:
        pad_xy = LAYOUT["pads_departure"][pad]
        return departure_track(sim_uid, pad_xy, LAYOUT["departure_exit"],
                               t_takeoff=t_run, label=req.label)
    pad_xy = LAYOUT["pads_arrival"][pad]
    return arrival_track(sim_uid, LAYOUT["arrival_fix"], pad_xy,
                         t_touchdown=t_run, label=req.label)


def _pick_schedule(sc: Scenario, sched) -> Dict[str, Assignment]:
    return {(a.uid, a.kind): a for a in sched.assignments}


def run_case_sim(sc: Scenario, before, after,
                 progress: bool = False) -> Optional[dict]:
    """抽 2~3 架排队前后的 UAV 真飞对比（同一个物理架次飞两遍）。

    排队前按请求时刻真飞（起降位按不排队基线分配），排队后按排定时刻
    真飞；分别统计实际离地/接地时刻与相邻间隔。
    """
    from .sim import run_fleet  # 延迟导入：--no-sim 时不加载 mujoco

    picks = select_sim_ops(sc.requests)
    if len(picks) < 2:
        return None
    before_map = _pick_schedule(sc, before)
    after_map = _pick_schedule(sc, after)

    def build(use_after: bool) -> List[Track]:
        src = after_map if use_after else before_map
        tracks = []
        for r in picks:
            a = src[(r.uid, r.kind)]
            tracks.append(_pick_track(r, a.t_scheduled, a.pad))
        # 统一时间原点：进场航迹可能早于 0 起始，整体平移保持相对间隔
        shift = -min(t.start_time for t in tracks)
        if shift > 1e-9:
            for t in tracks:
                for s in t.segments:
                    s.t0 += shift
                    s.t1 += shift
                t._starts = [s.t0 for s in t.segments]
                t.op_time += shift
        return tracks

    if progress:
        print("[%s] 排队前真飞：%s" % (sc.name, [r.uid for r in picks]))
    res_before = run_fleet(build(False), progress=progress)
    if progress:
        print("[%s] 排队后真飞：%s" % (sc.name, [r.uid for r in picks]))
    res_after = run_fleet(build(True), progress=progress)

    ev_b = res_before.op_events()
    ev_a = res_after.op_events()

    def _strip(events: dict) -> list:
        out = []
        for e in events.values():
            e = dict(e)
            e["uid"] = e["uid"].split("#", 1)[0]
            out.append(e)
        out.sort(key=lambda e: (e["planned"] if e["actual"] is not None
                                else 1e18, e["uid"]))
        return out

    def _err_map(errs: dict) -> dict:
        return {u.split("#", 1)[0]: round(v, 2) for u, v in errs.items()}

    return dict(
        uids=[r.uid for r in picks],
        picks=[dict(uid=r.uid, kind=r.kind, t_request=r.t_request,
                    priority=r.priority, label=r.label) for r in picks],
        before=dict(events=_strip(ev_b),
                    dep_gaps=res_before.interval_gaps(DEPARTURE),
                    arr_gaps=res_before.interval_gaps(ARRIVAL),
                    max_track_error=_err_map(res_before.max_track_error),
                    wall_time=round(res_before.wall_time, 2)),
        after=dict(events=_strip(ev_a),
                   dep_gaps=res_after.interval_gaps(DEPARTURE),
                   arr_gaps=res_after.interval_gaps(ARRIVAL),
                   max_track_error=_err_map(res_after.max_track_error),
                   wall_time=round(res_after.wall_time, 2)),
    )


# --------------------------------------------------------------------------- #
# 输出
# --------------------------------------------------------------------------- #
def to_json(results: Sequence[CaseResult], capacity: dict,
            path: str) -> str:
    payload = dict(
        package="uav_vertiport_queue",
        scenarios=[_case_dict(r) for r in results],
        capacity=capacity,
    )
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def _case_dict(r: CaseResult) -> dict:
    sc = r.scenario
    return dict(
        name=sc.name, title=sc.title, description=sc.description,
        strategy=sc.strategy, n_requests=len(sc.requests),
        requests=[dict(uid=q.uid, kind=q.kind, t_request=q.t_request,
                       priority=q.priority, label=q.label)
                  for q in sc.requests],
        before=r.metrics_before, after=r.metrics_after,
        before_assignments=[a.as_dict() for a in r.before.assignments],
        after_assignments=[a.as_dict() for a in r.after.assignments],
        pad_conflict_pairs=find_conflicts(r.before.assignments, sc.config),
        sim=r.sim,
    )


def format_report(r: CaseResult) -> str:
    s = r.scenario
    mb, ma = r.metrics_before, r.metrics_after
    lines = ["=" * 72,
             "%s — %s（%s 策略，%d 架次）" % (s.name, s.title, s.strategy,
                                            mb["requested"]),
             s.description,
             "-" * 72,
             "%-28s %12s %12s" % ("指标", "排队前", "排队后")]
    rows = [
        ("起降位占用冲突（对）", mb["pad_conflicts"], ma["pad_conflicts"]),
        ("平均等待 / s", mb["avg_wait"], ma["avg_wait"]),
        ("最大等待 / s", mb["max_wait"], ma["max_wait"]),
        ("总延误 / s", mb["delay_total"], ma["delay_total"]),
        ("最小离场间隔 / s", mb["min_dep_interval"], ma["min_dep_interval"]),
        ("最小进场间隔 / s", mb["min_arr_interval"], ma["min_arr_interval"]),
        ("离场间隔违规次数", mb["dep_interval_violations"],
         ma["dep_interval_violations"]),
        ("进场间隔违规次数", mb["arr_interval_violations"],
         ma["arr_interval_violations"]),
        ("最大排队长度 / 架", mb["max_queue_length"], ma["max_queue_length"]),
    ]
    for name, b, a in rows:
        lines.append("%-28s %12s %12s" % (name, b, a))
    lines.append("逐架次排定结果（前 8 条）：")
    lines.append("  %-10s %-10s %-5s %10s %10s %8s" %
                 ("机号", "类型", "位", "请求", "排定", "等待s"))
    for a in sorted(r.after.assignments, key=lambda a: a.t_scheduled)[:8]:
        lines.append("  %-10s %-10s %-5s %10.1f %10.1f %8.1f" %
                     (a.uid, a.kind, a.pad, a.t_request, a.t_scheduled,
                      a.delay))
    if r.sim:
        lines.append("MuJoCo 闭环抽飞（%s）：" % ", ".join(r.sim["uids"]))
        for phase, key in (("排队前", "before"), ("排队后", "after")):
            sim = r.sim[key]
            lines.append("  %s：离场实际间隔 %s s，进场实际间隔 %s s，"
                         "最大跟踪误差 %.2f m"
                         % (phase,
                            sim["dep_gaps"] or "—",
                            sim["arr_gaps"] or "—",
                            max(sim["max_track_error"].values() or [0.0])))
            for e in sim["events"]:
                lines.append("    %-8s %-10s 计划 %8.2f 实际 %8s 偏差 %s s"
                             % (e["uid"], e["kind"], e["planned"],
                                e["actual"], e["error"]))
    return "\n".join(lines)
