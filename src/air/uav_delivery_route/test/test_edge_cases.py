#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Edge cases: energy model properties, payload/energy limits, dispatch logic."""
from __future__ import annotations

import math

from uav_delivery_route.core.cases import Case, Station
from uav_delivery_route.core.checker import check_plan
from uav_delivery_route.core.energy import (RETURN_MARGIN, EnergyModel,
                                            needs_early_return)
from uav_delivery_route.core.planner import (RouteConfig, decide_next,
                                             plan_routes, sector_clusters,
                                             split_sorties)

CFG = RouteConfig()
MODEL = EnergyModel()


def test_energy_linear_in_load():
    # P(m1 + m2) - P(m1) == P(m2) - P(0)：功率对载重严格线性
    for m1, m2 in ((0.0, 1.0), (0.6, 0.9), (1.2, 1.3)):
        assert math.isclose(MODEL.power_w(m1 + m2) - MODEL.power_w(m1),
                            MODEL.power_w(m2) - MODEL.power_w(0.0), rel_tol=1e-12)
    assert MODEL.power_w(2.5) == MODEL.p_base_w + MODEL.load_coef_w_per_kg * 2.5


def test_capacity_matches_endurance_definition():
    # 满载 2.5 kg 巡航恰好把电池用 18 min 用完
    frac = MODEL.energy_frac(2.5, MODEL.endurance_s)
    assert math.isclose(frac, 1.0, rel_tol=1e-12)
    assert math.isclose(MODEL.capacity_j, 85.0 * 1080.0, rel_tol=1e-12)


def test_single_station_plan():
    case = Case("tiny", "one station", stations=(Station("S1", 300.0, 100.0, 1.0),))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count == 1 and plan.swap_count == 0
    assert check_plan(plan, case, MODEL, CFG) == []


def test_station_exactly_at_payload_cap():
    case = Case("cap", "2.5 kg package", stations=(Station("S1", 200.0, 0.0, 2.5),))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sorties[0].payload_max_kg == 2.5
    assert check_plan(plan, case, MODEL, CFG) == []


def test_station_over_cap_flagged_unservable():
    case = Case("over", "3 kg package", stations=(Station("S1", 200.0, 0.0, 3.0),))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.unservable == ["S1"]
    assert check_plan(plan, case, MODEL, CFG)  # 违反清单非空


def test_beyond_range_station_still_planned_and_flagged():
    # 单程 11 km 超出满电往返能量（去程带载 + 回程空载 108.8 kJ > 91.8 kJ）：
    # 规划不死循环、站点仍被排入，checker 如实报续航违规
    case = Case("far", "out of range", stations=(Station("S1", 11000.0, 0.0, 0.5),))
    plan = plan_routes(case, CFG, MODEL)
    served = [s.sid for so in plan.sorties for s in so.stations]
    assert served == ["S1"]
    assert any("endurance" in v for v in check_plan(plan, case, MODEL, CFG))


def test_return_margin_triggers_before_hard_limit():
    # 换电判据下，任一架次剩余电量始终 >= 1.0 x 返航所需（min_home_margin >= 1）
    case = Case("wide", "mid-distance ring",
                stations=tuple(Station("S%d" % i,
                                       2500.0 * math.cos(a), 2500.0 * math.sin(a), 0.8)
                               for i, a in enumerate(
                                   [k * 2 * math.pi / 8 for k in range(8)])))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count >= 2
    assert all(so.min_home_margin >= 1.0 for so in plan.sorties)
    assert all(so.energy_frac <= 1.0 for so in plan.sorties)


def test_needs_early_return_threshold():
    need = MODEL.leg_frac(1000.0, 0.0, 9.0)
    assert needs_early_return(MODEL, RETURN_MARGIN * need - 1e-9, 1000.0, 0.0, 9.0)
    assert not needs_early_return(MODEL, RETURN_MARGIN * need + 1e-9, 1000.0, 0.0, 9.0)


def test_single_sector_groups_everything():
    clusters = sector_clusters(_mk(), (0.0, 0.0), 1)
    assert len(clusters) == 1 and len(clusters[0]) == 3


def _mk():
    return (Station("A", 100.0, 0.0, 0.5), Station("B", 0.0, 100.0, 0.5),
            Station("C", -100.0, 0.0, 0.5))


def test_split_with_empty_order():
    assert split_sorties((0.0, 0.0), [], MODEL, CFG) == []


def test_decide_next_deliver_swap_done():
    case = Case("disp", "dispatch", stations=(
        Station("S1", 400.0, 0.0, 1.0), Station("S2", 800.0, 0.0, 1.0)))
    plan = plan_routes(case, CFG, MODEL)
    visited: set = set()
    d = decide_next(plan, visited, (0.0, 0.0), 1.0, MODEL, CFG)
    assert d["action"] == "deliver" and d["target"]["sid"] == "S1"
    # 电量告急：低于 1.25 x 返航所需 -> 建议回中心换电
    d = decide_next(plan, {"S1"}, (400.0, 0.0), 0.05, MODEL, CFG)
    assert d["action"] == "return_for_swap" and d["target"]["sid"] == "depot"
    # 全部服务完 -> done
    d = decide_next(plan, {"S1", "S2"}, (0.0, 0.0), 1.0, MODEL, CFG)
    assert d["action"] == "done" and d["done"]
