#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Feasibility checker: endurance / payload / coverage, returns violation list.

单行英文摘要 + 中文正文。
对 FleetPlan 做三类硬检查：每个架次总能耗不超电池容量（续航不超）、
任意时刻载重不超 2.5 kg 上限、所有站点恰好被服务一次。输出违反清单，
正常规划下应为空。
"""
from __future__ import annotations

from typing import List

from .cases import Case
from .energy import EnergyModel
from .planner import FleetPlan, RouteConfig


def check_plan(plan: FleetPlan, case: Case, model: EnergyModel | None = None,
               cfg: RouteConfig | None = None) -> List[str]:
    """Return a list of violation strings; empty means the plan is feasible."""
    model = model or EnergyModel()
    cfg = cfg or RouteConfig()
    bad: List[str] = []

    # 1) 站点覆盖：每个站恰好一次（不可服务的超重站单独记账）
    served = [s.sid for so in plan.sorties for s in so.stations]
    expect = {s.sid for s in case.stations if s.weight_kg <= cfg.payload_cap_kg + 1e-9}
    for sid in sorted(expect):
        n = served.count(sid)
        if n == 0:
            bad.append("coverage: station %s not served" % sid)
        elif n > 1:
            bad.append("coverage: station %s served %d times" % (sid, n))
    extra = set(served) - expect
    for sid in sorted(extra):
        bad.append("coverage: station %s served but over payload cap" % sid)

    for so in plan.sorties:
        tag = "sortie %d" % so.index
        # 2) 续航：整架次（含返航）能耗不超电池容量
        if so.energy_frac > 1.0 + 1e-9:
            bad.append("%s: endurance exceeded (%.1f%% of battery)"
                       % (tag, so.energy_frac * 100.0))
        # 3) 载重：任意时刻机上重量不超上限
        if so.payload_max_kg > cfg.payload_cap_kg + 1e-9:
            bad.append("%s: payload %.2f kg exceeds cap %.2f kg"
                       % (tag, so.payload_max_kg, cfg.payload_cap_kg))
        # 4) 单站包裹本身超重则该站不可服务
        for s in so.stations:
            if s.weight_kg > cfg.payload_cap_kg + 1e-9:
                bad.append("%s: station %s package %.2f kg over cap"
                           % (tag, s.sid, s.weight_kg))
    for sid in plan.unservable:
        bad.append("unservable: station %s package over payload cap" % sid)
    return bad
