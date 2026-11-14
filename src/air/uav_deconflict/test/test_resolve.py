#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解脱算法测试：无冲突不动、医疗机不被延误、禁用地等时只能用几何手段。"""
import pytest

from uav_deconflict.core.conflict import Separation, detect
from uav_deconflict.core.plan import Plan
from uav_deconflict.core.resolve import Limits, report_rows, resolve_fleet
from uav_deconflict.core.scenarios import get_scenario


def test_no_conflict_means_no_action():
    plans = [
        Plan("A", [(0, 0), (100, 0)], alt=40.0, speed=10.0),
        Plan("B", [(0, 200), (100, 200)], alt=40.0, speed=10.0),
    ]
    res = resolve_fleet(plans, Separation())
    assert res.solved
    assert res.actions == []
    assert res.iterations == 1


def test_corridor_solved_and_medical_untouched():
    sc = get_scenario("corridor")
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)
    assert res.solved
    assert not any(a.uid == "UAV-4" for a in res.actions), "医疗急救机不应被延误"
    # 医疗机的航迹必须与原计划完全一致
    base = {p.uid: p for p in sc.plans}
    assert res.plans["UAV-4"].t0 == pytest.approx(base["UAV-4"].t0)
    assert res.plans["UAV-4"].alt == pytest.approx(base["UAV-4"].alt)


def test_slot_scenario_forbids_ground_delay():
    sc = get_scenario("slot")
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)
    assert res.solved
    assert all(a.kind != "delay" for a in res.actions)
    # 至少用到了几何手段
    assert any(a.kind in ("altitude", "lateral") for a in res.actions)


def test_delay_respects_max_delay():
    limits = Limits(max_delay=15.0)
    sc = get_scenario("corridor")
    res = resolve_fleet(sc.plans, sc.sep, limits=limits)
    base = {p.uid: p for p in sc.plans}
    for uid, plan in res.plans.items():
        assert plan.t0 - base[uid].t0 <= limits.max_delay + 1e-9


def test_protected_aircraft_is_not_moved():
    sc = get_scenario("metro")
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits, protected={"UAV-3"})
    base = {p.uid: p for p in sc.plans}
    assert not any(a.uid == "UAV-3" for a in res.actions)
    assert res.plans["UAV-3"].t0 == pytest.approx(base["UAV-3"].t0)


def test_resolution_never_shortens_mission():
    sc = get_scenario("metro")
    base = {p.uid: p for p in sc.plans}
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)
    for uid, plan in res.plans.items():
        assert plan.end_time >= base[uid].end_time - 1e-9


def test_report_rows_cover_all_aircraft():
    sc = get_scenario("corridor")
    res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)
    rows = report_rows(res, {p.uid: p for p in sc.plans})
    assert len(rows) == len(sc.plans)
    assert {r["uid"] for r in rows} == {p.uid for p in sc.plans}
    assert all(r["actions"] for r in rows if r["delay"] > 0 or r["alt_new"] != r["alt_base"])


def test_all_scenarios_solved():
    for name in ("crossing", "corridor", "metro", "slot"):
        sc = get_scenario(name)
        res = resolve_fleet(sc.plans, sc.sep, limits=sc.limits)
        assert res.solved, "%s 仍有 %d 段冲突" % (name, len(res.remaining))
        assert detect([res.plans[p.uid] for p in sc.plans], sc.sep) == []
