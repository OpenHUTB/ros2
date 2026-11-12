#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""内置场景单元测试：规模、急救优先级、可排队性。"""
from uav_vertiport_queue.core.pipeline import run_case
from uav_vertiport_queue.core.queue import (QueueConfig, assign_pads_no_wait,
                                            find_conflicts, schedule)
from uav_vertiport_queue.core.scenarios import SCENARIOS, get_scenario


def test_scenario_catalog():
    assert set(SCENARIOS) == {"light", "rush", "medical", "saturation"}


def test_scenario_sizes():
    assert len(get_scenario("light").requests) == 8
    assert len(get_scenario("rush").requests) == 16
    assert len(get_scenario("medical").requests) == 12
    assert len(get_scenario("saturation").requests) == 20


def test_uav_fleet_within_6_to_10():
    for name in ("light", "rush", "medical", "saturation"):
        uids = {r.uid for r in get_scenario(name).requests}
        assert 6 <= len(uids) <= 10, "%s 机队 %d 架" % (name, len(uids))


def test_medical_has_priority_request():
    sc = get_scenario("medical")
    med = [r for r in sc.requests if r.priority == 10]
    assert med, "医疗场景必须含 priority=10 的急救请求"
    assert sc.strategy == "priority"


def test_all_scenarios_schedulable():
    for name in SCENARIOS:
        sc = get_scenario(name)
        cfg = QueueConfig()
        before = assign_pads_no_wait(sc.requests, cfg)
        after = schedule(sc.requests, cfg, sc.strategy)
        # 排队前确实存在起降位占用冲突（说明场景有排队需求）
        assert len(find_conflicts(before.assignments, cfg)) > 0, name
        # 排队后冲突清零
        assert find_conflicts(after.assignments, cfg) == [], name


def test_run_case_metrics_consistent():
    r = run_case("light", sim=False)
    assert r.metrics_after["scheduled"] == r.metrics_before["requested"]
    assert r.metrics_after["pad_conflicts"] == 0
    assert r.metrics_after["avg_wait"] >= r.metrics_before["avg_wait"]
