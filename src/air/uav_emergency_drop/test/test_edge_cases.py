#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Edge cases: ballistics values, degenerate inputs, trigger feasibility,
wind compensation."""
import math

import numpy as np
import pytest

from uav_emergency_drop.core.ballistics import (BallisticsCfg,
                                                conditions_table,
                                                fall_time, impact_point,
                                                make_release_plan,
                                                overflight_target,
                                                release_window,
                                                scatter_sigma, wind_drift)
from uav_emergency_drop.core.planner import CargoCfg, DemandPoint, \
    plan_missions, select_points
from uav_emergency_drop.core.trigger import check_leg, evaluate_drop


def test_empty_demand_points():
    selected, missions = plan_missions([], CargoCfg())
    assert selected == [] and missions == []


def test_point_exceeding_capacity_skipped():
    ps = [DemandPoint("HUGE", 0, 0, 20, 5), DemandPoint("OK", 1, 0, 4, 3)]
    selected, missions = plan_missions(ps, CargoCfg(capacity=12))
    assert selected == ["OK"]
    assert missions[0].pids == ["OK"]


def test_zero_capacity_selects_nothing():
    ps = [DemandPoint("A", 0, 0, 3, 5)]
    assert select_points(ps, 0) == []


def test_fall_time_values():
    assert fall_time(20.0) == pytest.approx(math.sqrt(40.0 / 9.81))
    assert fall_time(50.0) == pytest.approx(math.sqrt(100.0 / 9.81))
    with pytest.raises(ValueError):
        fall_time(0.0)


def test_sigma_scales_with_height():
    assert scatter_sigma(20.0) == pytest.approx(3.0)
    assert scatter_sigma(50.0) == pytest.approx(7.5)
    assert scatter_sigma(50.0) / scatter_sigma(20.0) == pytest.approx(2.5)


def test_wind_drift_formula():
    # 落点风偏 = w * t_fall * 0.8
    t = fall_time(20.0)
    d = wind_drift((8.0, 0.0), 20.0)
    assert d[0] == pytest.approx(8.0 * t * 0.8)
    assert d[1] == pytest.approx(0.0)


def test_impact_no_wind_equals_forward_throw():
    plan = make_release_plan(30.0, 8.0, (0.0, 0.0))
    imp = impact_point((100.0, 200.0), (8.0, 0.0), plan)
    assert imp[0] == pytest.approx(100.0 + 8.0 * plan.t_fall)
    assert imp[1] == pytest.approx(200.0)


def test_conditions_table_covers_grid():
    rows = conditions_table()
    assert len(rows) == 9  # 3 heights x 3 speeds
    hs = {(r["h_m"], r["speed_mps"]) for r in rows}
    assert (20, 6) in hs and (50, 10) in hs
    # 同一高度提前量随速度单调增
    leads = [r["lead_m"] for r in rows if r["h_m"] == 30]
    assert leads == sorted(leads)


def test_release_window_infeasible_short_leg():
    # 提前量 > 段长 → 不可行并给降速/绕行建议
    plan = make_release_plan(50.0, 10.0)  # lead = 10 * 3.19 = 31.9 m
    win = check_leg((0.0, 0.0), (20.0, 0.0), (20.0, 0.0), plan)
    assert not win.feasible
    assert "slow" in win.advice or "detour" in win.advice
    assert win.s_enter >= 0 or "extend" in win.advice


def test_release_window_contains_ideal_point():
    plan = make_release_plan(30.0, 8.0)
    win = check_leg((0.0, 0.0), (200.0, 0.0), (200.0, 0.0), plan)
    assert win.feasible
    assert win.s_enter <= win.s_ideal <= win.s_exit
    assert win.s_ideal == pytest.approx(200.0 - plan.lead_m, abs=0.5)
    assert win.cross_error_m == pytest.approx(0.0, abs=1e-6)


def test_evaluate_drop_fires_in_window():
    plan = make_release_plan(30.0, 8.0)
    seg_a, seg_b, tgt = (0.0, 0.0), (200.0, 0.0), (200.0, 0.0)
    s_ideal = 200.0 - plan.lead_m
    pos = (s_ideal, 0.0)
    v = evaluate_drop(pos, (8.0, 0.0), seg_a, seg_b, tgt, plan)
    assert v.release_now
    imp = impact_point(pos, (8.0, 0.0), plan)
    assert v.predicted_impact[0] == pytest.approx(imp[0])
    # 区间外不投
    out = evaluate_drop((10.0, 0.0), (8.0, 0.0), seg_a, seg_b, tgt, plan)
    assert not out.release_now
    # 已投过不重复投
    done = evaluate_drop(pos, (8.0, 0.0), seg_a, seg_b, tgt, plan,
                         released=True)
    assert not done.release_now


def test_wind_compensated_aim_lands_on_target():
    # 逆风偏移越顶点后，在提前量处释放，解析落点应回到真实目标
    plan = make_release_plan(20.0, 6.0, (8.0, 0.0))
    tgt = np.array([300.0, 100.0])
    wp = overflight_target(tgt, plan)  # target - drift
    rel = wp - np.array([plan.lead_m, 0.0])  # 理想释放点 = 越顶点 - 前抛
    imp = impact_point(rel, (6.0, 0.0), plan)
    assert imp == pytest.approx(tgt, abs=1e-9)


def test_release_window_rejects_lateral_offset():
    plan = make_release_plan(20.0, 6.0, (0.0, 0.0))
    # 航线横向偏离目标 30 m，超出可接受半径 6 m
    win = release_window((0.0, 30.0), (200.0, 30.0), (200.0, 0.0), plan)
    assert not win.feasible
    assert "realign" in win.advice
