#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Edge cases: energy model properties, payload/energy limits, dispatch logic."""
from __future__ import annotations

import math

from uav_delivery_route.core.cases import Case, Station
from uav_delivery_route.core.checker import check_plan
from uav_delivery_route.core.energy import (RETURN_MARGIN, EnergyModel,
                                            needs_early_return)
from uav_delivery_route.core.planner import (FleetPlan, RouteConfig,
                                             _sortie_cost, decide_next,
                                             plan_routes, sector_clusters,
                                             sortie_home_margins,
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
    # 电量告急：低于 1.25 x 返航所需 -> 建议回中心换电（已离仓在途）
    d = decide_next(plan, {"S1"}, (400.0, 0.0), 0.05, MODEL, CFG,
                    departed=True)
    assert d["action"] == "return_for_swap" and d["target"]["sid"] == "depot"
    # 全部服务完 -> done
    d = decide_next(plan, {"S1", "S2"}, (0.0, 0.0), 1.0, MODEL, CFG)
    assert d["action"] == "done" and d["done"]


def test_split_recomputes_full_load_trajectory():
    # 评审反例：两个 1.2 kg 远站相距 1 m。旧账目把"已投递重量"当机上
    # 剩余，两站被并进一个架次、报告裕量 1.28；按满载轨迹重算，第一站
    # 投完的真实裕量只有 0.96，必须拆成两个架次
    A = Station("A", 5700.0, 0.0, 1.2)
    B = Station("B", 5701.0, 0.0, 1.2)
    assert min(sortie_home_margins([A, B], (0.0, 0.0), MODEL, CFG)) < CFG.return_margin

    case = Case("split", "two far 1.2 kg stations", stations=(A, B))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count == 2, plan.to_dict()
    assert check_plan(plan, case, MODEL, CFG) == []
    for so in plan.sorties:
        real = min(sortie_home_margins(so.stations, plan.depot, MODEL, CFG))
        assert real >= CFG.return_margin - 1e-9
        assert math.isclose(real, so.min_home_margin, rel_tol=1e-6)


def test_checker_recomputes_margin_independently():
    # checker 不信任规划器报的 min_home_margin：手工构造"两远站塞进一个
    # 架次、报告裕量无穷大"的账本，独立复算必须报违反
    A = Station("A", 5700.0, 0.0, 1.2)
    B = Station("B", 5701.0, 0.0, 1.2)
    s = _sortie_cost([A, B], (0.0, 0.0), MODEL, CFG)
    s.min_home_margin = math.inf
    plan = FleetPlan("fake", (0.0, 0.0), [s], 0.0, 0.0, 0)
    bad = check_plan(plan, Case("fake", "x", stations=(A, B)), MODEL, CFG)
    assert any("home margin" in v for v in bad)


def test_decide_next_returns_to_depot_between_sorties():
    # 评审复现：第一架次送完停在现场，不能直接去送下一架次的站，
    # 必须先回中心；全部送完也要回中心才算 done
    A = Station("A", 5700.0, 0.0, 1.2)
    B = Station("B", 5701.0, 0.0, 1.2)
    case = Case("ret", "two far stations", stations=(A, B))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count == 2
    d = decide_next(plan, {"A"}, (5700.0, 0.0), 0.8, MODEL, CFG)
    assert d["action"] == "return_to_depot" and d["target"]["sid"] == "depot"
    # 回到中心、电量不够飞下一架次计划账目 -> 先换电
    e_next = plan.sorties[1].energy_frac
    d = decide_next(plan, {"A"}, (0.0, 0.0), min(0.99, e_next * 0.5),
                    MODEL, CFG)
    assert d["action"] == "swap"
    # 电量恢复 -> 开始下一架次
    d = decide_next(plan, {"A"}, (0.0, 0.0), 1.0, MODEL, CFG)
    assert d["action"] == "deliver" and d["sortie"] == 1
    # 全部送完但人不在中心 -> 回中心而不是 done
    d = decide_next(plan, {"A", "B"}, (5701.0, 0.0), 0.5, MODEL, CFG)
    assert d["action"] == "return_to_depot"
    d = decide_next(plan, {"A", "B"}, (0.0, 0.0), 0.5, MODEL, CFG)
    assert d["action"] == "done" and d["done"]


def test_decide_next_first_sortie_en_route_delivers():
    # 评审复现：单站满电，(0,0) 给 deliver；离仓在途不能被误判成
    # "新架次还没出发"而要求返仓——首次投递必须能完成。
    # departed 语义 = 本架次已装货离仓，节点在发出 deliver 时置位。
    A = Station("A", 100.0, 0.0, 0.5)
    case = Case("one", "single station", stations=(A,))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count == 1
    # 在中心：装货出发
    d = decide_next(plan, set(), (0.0, 0.0), 1.0, MODEL, CFG)
    assert d["action"] == "deliver" and d["target"]["sid"] == "A"
    # 刚离开 2 m 半径（节点已把 departed 置 True）：继续送，不许返仓
    d = decide_next(plan, set(), (3.0, 0.0), 1.0, MODEL, CFG, departed=True)
    assert d["action"] == "deliver" and d["target"]["sid"] == "A"
    # 到站上空：仍然 deliver（由节点悬停确认后标记 visited）
    d = decide_next(plan, set(), (100.0, 0.0), 1.0, MODEL, CFG, departed=True)
    assert d["action"] == "deliver"
    # 对照：同样位置但没装货离仓（上一架次被拉走）-> 必须先回中心
    d = decide_next(plan, set(), (3.0, 0.0), 1.0, MODEL, CFG, departed=False)
    assert d["action"] == "return_to_depot"


def test_decide_next_mid_sortie_return_swaps_then_resumes():
    # 评审复现：两站各 0.5 kg，送完 A 后中途低电返仓。到仓后必须进入
    # swap（等待恢复），恢复后恢复同一架次继续送 B——而不是在原地
    # 反复 return_for_swap（目标还是自己所在的仓库）。
    A = Station("A", 100.0, 0.0, 0.5)
    B = Station("B", 200.0, 0.0, 0.5)
    case = Case("two", "two stations one sortie", stations=(A, B))
    plan = plan_routes(case, CFG, MODEL)
    assert plan.sortie_count == 1
    # A、B 之间电量告急（低于 1.25 x 返航所需 + 去程）-> return_for_swap
    d = decide_next(plan, {"A"}, (150.0, 0.0), 0.01, MODEL, CFG, departed=True)
    assert d["action"] == "return_for_swap" and d["target"]["sid"] == "depot"
    # 已经回到中心（哪怕本架次已 started）-> swap，等待换电
    d = decide_next(plan, {"A"}, (0.0, 0.0), 0.01, MODEL, CFG, departed=True)
    assert d["action"] == "swap" and d["sortie"] == 0
    # 换电完成 -> 恢复同一架次（sortie 0）继续送 B
    d = decide_next(plan, {"A"}, (0.0, 0.0), 1.0, MODEL, CFG, departed=True)
    assert d["action"] == "deliver" and d["sortie"] == 0
    assert d["target"]["sid"] == "B"
    # B 送完回到中心 -> done
    d = decide_next(plan, {"A", "B"}, (0.0, 0.0), 0.9, MODEL, CFG, departed=True)
    assert d["action"] == "done" and d["done"]
