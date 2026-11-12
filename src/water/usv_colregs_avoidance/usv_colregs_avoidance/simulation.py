#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四个会遇场景：同一初始态势下「不避让」与「按 COLREGs 避让」的对比。

避让过程是**闭环判定**：每一步都重新计算 CPA/TCPA 与会遇局面，但只执行一次机动
（模拟驾驶员打一次舵后保持），之后持续监视。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .colregs import (
    DEFAULT_THRESHOLDS,
    Decision,
    ShipState,
    Thresholds,
    apply_course_offset,
    decide,
    relative_geometry,
)

DT = 1.0


@dataclass
class ScenarioResult:
    name: str
    aspect: str
    rule: str
    action: str
    min_distance_compliant: float
    min_distance_inaction: float
    course_offset_deg: float
    initial_distance: float
    duration_s: float
    dt: float
    intervened: bool
    trace_compliant: list = field(default_factory=list)


def _step(state: ShipState, dt: float) -> ShipState:
    return ShipState(state.x + state.speed * math.cos(state.heading) * dt,
                     state.y + state.speed * math.sin(state.heading) * dt,
                     state.heading, state.speed)


def _simulate(own0: ShipState, other0: ShipState, duration: float,
              apply_rules: bool, th: Thresholds, dt: float = DT):
    own, other = own0, other0
    committed = False
    executed = Decision("hold", 0.0, 1.0, "none", "-")
    min_dist = math.hypot(other.x - own.x, other.y - own.y)
    prev_dist = min_dist
    trace = []

    for k in range(int(duration / dt)):
        enc = relative_geometry(own, other, th)
        if apply_rules:
            decision = decide(enc, th)
            # 让路船执行机动；直航船只在 Rule 17(b) 的紧急情况下才动。
            give_way = enc.aspect in ("head_on", "crossing_give_way",
                                      "overtaking_own")
            may_act = (give_way and decision.action != "hold") or                 decision.action == "emergency_starboard"
            if not committed and may_act:
                own = apply_course_offset(own, decision.course_offset_deg,
                                          decision.speed_scale)
                committed = True
                executed = decision
        dist = math.hypot(other.x - own.x, other.y - own.y)
        min_dist = min(min_dist, dist)
        if k % 5 == 0:
            trace.append((k * dt, own.x, own.y, other.x, other.y))

        # 已经拉开且距离连续增大，可以提前结束
        if k > 10 and enc.aspect == "opening" and dist > prev_dist:
            break
        prev_dist = dist
        own, other = _step(own, dt), _step(other, dt)

    return min_dist, executed, trace, committed


def run_scenario(name: str, own: ShipState, other: ShipState,
                 duration: float = 400.0, th: Thresholds | None = None,
                 dt: float = DT) -> ScenarioResult:
    th = th or DEFAULT_THRESHOLDS
    enc = relative_geometry(own, other, th)
    compliant_dist, decision, trace, committed = _simulate(
        own, other, duration, True, th, dt)
    inaction_dist, _d, _t, _c = _simulate(own, other, duration, False, th, dt)
    return ScenarioResult(
        name=name,
        aspect=enc.aspect,
        rule=decision.rule,
        action=decision.action,
        min_distance_compliant=compliant_dist,
        min_distance_inaction=inaction_dist,
        course_offset_deg=decision.course_offset_deg,
        initial_distance=enc.range_m,
        duration_s=duration,
        dt=dt,
        intervened=committed,
        trace_compliant=trace,
    )


def default_scenarios(th: Thresholds | None = None) -> list:
    own = ShipState(0.0, 0.0, 0.0, 5.0)
    return [
        # 对遇：目标正前方 1000 m，反向同速
        run_scenario("head_on", own, ShipState(1000.0, 0.0, math.pi, 5.0), th=th),
        # 交叉（本船让路）：目标在右舷、向南行驶
        run_scenario("crossing_give_way", own,
                     ShipState(800.0, 600.0, -math.pi / 2, 5.0), th=th),
        # 被追越：目标从正后方以 8 m/s 追上来
        run_scenario("overtaken", own, ShipState(-800.0, 0.0, 0.0, 8.0), th=th),
        # 本船追越：正前方同向、目标更慢
        run_scenario("overtaking_own", own, ShipState(1000.0, 0.0, 0.0, 2.0),
                     th=th),
        # 同向更快、正在拉开：不应触发任何动作
        run_scenario("opening", own, ShipState(1000.0, 0.0, 0.0, 10.0), th=th),
    ]


def format_report(results) -> str:
    lines = ["COLREGs 会遇判定与避让效果（闭环判定，单次机动）", "-" * 78]
    lines.append("%-18s %-18s %-12s %10s %10s" %
                 ("scenario", "initial aspect", "rule", "d_min(避让)", "d_min(不动)"))
    for r in results:
        lines.append("%-18s %-18s %-12s %10.1f %10.1f" %
                     (r.name, r.aspect, r.rule,
                      r.min_distance_compliant, r.min_distance_inaction))
    lines.append("")
    for r in results:
        lines.append("[%s] 初始 %s（%.0f m），建议 %s（%+.0f°），机动=%s" %
                     (r.name, r.aspect, r.initial_distance, r.action,
                      r.course_offset_deg, "是" if r.intervened else "否"))
    return "\n".join(lines)
