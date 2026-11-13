#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Planner basics: clustering, NN, 2-opt, battery split, coverage."""
from __future__ import annotations

import math

from uav_delivery_route.core.cases import (CASE_FARM_DENSE, CASE_FARM_HEAVY,
                                          CASE_FARM_SPARSE, Station, get_case)
from uav_delivery_route.core.checker import check_plan
from uav_delivery_route.core.energy import EnergyModel
from uav_delivery_route.core.planner import (FleetPlan, RouteConfig,
                                             nearest_neighbor, plan_routes,
                                             sector_clusters, split_sorties,
                                             tour_length, two_opt)

CFG = RouteConfig()
MODEL = EnergyModel()


def _plan(case):
    return plan_routes(case, CFG, MODEL)


def test_sector_clusters_cover_all():
    clusters = sector_clusters(CASE_FARM_SPARSE.stations, (0.0, 0.0), 3)
    got = [s.sid for c in clusters for s in c]
    assert sorted(got) == sorted(s.sid for s in CASE_FARM_SPARSE.stations)
    assert len(got) == len(set(got))


def test_nn_order_visits_each_once():
    order = nearest_neighbor((0.0, 0.0), list(CASE_FARM_DENSE.stations))
    assert len(order) == 12
    assert len({s.sid for s in order}) == 12


def test_two_opt_never_worsens():
    order = nearest_neighbor((0.0, 0.0), list(CASE_FARM_SPARSE.stations))
    improved = two_opt((0.0, 0.0), order)
    assert tour_length((0.0, 0.0), improved) <= tour_length((0.0, 0.0), order) + 1e-6


def test_two_opt_fixes_crossed_tour():
    st = [Station("A", 0.0, 100.0, 0.5), Station("B", 100.0, 0.0, 0.5),
          Station("C", 0.0, -100.0, 0.5), Station("D", -100.0, 0.0, 0.5)]
    crossed = [st[0], st[2], st[1], st[3]]  # A->C 与 B->D 交叉
    fixed = two_opt((0.0, 0.0), crossed)
    assert tour_length((0.0, 0.0), fixed) < tour_length((0.0, 0.0), crossed) - 1.0


def test_dense_case_single_sortie():
    plan = _plan(CASE_FARM_DENSE)
    assert plan.sortie_count == 1
    assert plan.swap_count == 0


def test_sparse_case_multi_sortie_with_swap():
    plan = _plan(CASE_FARM_SPARSE)
    assert plan.sortie_count >= 2
    assert plan.swap_count == plan.sortie_count - 1


def test_heavy_case_needs_batching():
    plan = _plan(CASE_FARM_HEAVY)
    heavy = [s for s in CASE_FARM_HEAVY.stations if s.weight_kg >= 2.2]
    # 2.2 kg + 最轻 0.4 kg = 2.6 > 2.5，每个重包裹必须独占一个架次
    assert plan.sortie_count >= len(heavy)
    assert all(so.payload_max_kg <= CFG.payload_cap_kg + 1e-9 for so in plan.sorties)


def test_all_stations_served_exactly_once():
    for case in (CASE_FARM_DENSE, CASE_FARM_SPARSE, CASE_FARM_HEAVY):
        plan = _plan(case)
        served = sorted(s.sid for so in plan.sorties for s in so.stations)
        assert served == sorted(s.sid for s in case.stations)


def test_total_duration_accounts_hover_and_swap():
    plan = _plan(CASE_FARM_SPARSE)
    expect = sum(so.flight_s + so.hover_s for so in plan.sorties)
    expect += CFG.swap_s * (plan.sortie_count - 1)
    assert math.isclose(plan.total_duration_s, expect, rel_tol=1e-9)
    assert all(so.hover_s == CFG.hover_s * len(so.stations) for so in plan.sorties)


def test_checker_clean_on_plans():
    for case in (CASE_FARM_DENSE, CASE_FARM_SPARSE, CASE_FARM_HEAVY):
        plan = _plan(case)
        assert check_plan(plan, case, MODEL, CFG) == []


def test_energy_frac_within_battery_for_all_sorties():
    for case in (CASE_FARM_DENSE, CASE_FARM_SPARSE, CASE_FARM_HEAVY):
        for so in _plan(case).sorties:
            assert 0.0 < so.energy_frac <= 1.0
