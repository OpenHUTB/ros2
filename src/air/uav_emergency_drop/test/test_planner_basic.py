#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Planner basics: greedy selection, swap improvement, multi-sortie packing."""
import numpy as np
import pytest

from uav_emergency_drop.core.planner import (CargoCfg, DemandPoint,
                                             detour_via_bridge, greedy_select,
                                             plan_missions, route_order,
                                             segment_crosses_band,
                                             select_points, swap_improve)


def pts(*rows):
    return [DemandPoint(*r) for r in rows]


def test_greedy_respects_capacity():
    ps = pts(("A", 0, 0, 5, 3), ("B", 1, 0, 5, 3), ("C", 2, 0, 5, 3))
    picked = greedy_select(ps, capacity=12)
    assert len(picked) == 2  # 12 份只装得下两个 5 份点


def test_weight_prefers_urgency_and_demand():
    ps = pts(("HOT", 0, 0, 2, 5), ("BIG", 1, 0, 4, 4), ("COLD", 2, 0, 4, 1))
    picked = greedy_select(ps, capacity=6)
    assert picked[0] in ("HOT", "BIG")
    weights = {p.pid: p.weight for p in ps}
    assert all(weights[picked[i]] >= weights[picked[i + 1]]
               for i in range(len(picked) - 1))


def test_swap_improves_coverage():
    # 贪心只装下 A(6份)，交换改进后应换成 B+C(5+5=10 份)，加权覆盖更高
    ps = pts(("A", 0, 0, 6, 5), ("B", 1, 0, 5, 4), ("C", 2, 0, 5, 4))
    greedy = greedy_select(ps, capacity=10)
    improved = swap_improve(ps, greedy, capacity=10)
    assert _cov(ps, improved) > _cov(ps, greedy)
    load = sum(p.demand for p in ps if p.pid in improved)
    assert load <= 10


def _cov(ps, pids):
    m = {p.pid: p for p in ps}
    return sum(m[i].weight for i in pids)


def test_swap_never_regresses():
    rng = np.random.default_rng(7)
    for _ in range(20):
        ps = [DemandPoint("P%d" % i, float(i), 0.0, int(rng.integers(1, 5)),
                          int(rng.integers(1, 6))) for i in range(8)]
        cap = int(rng.integers(6, 14))
        greedy = greedy_select(ps, cap)
        improved = select_points(ps, cap)
        assert _cov(ps, improved) >= _cov(ps, greedy)
        load = sum(p.demand for p in ps if p.pid in improved)
        assert load <= cap


def test_multi_sortie_splits_with_reload():
    ps = pts(("A", 0, 0, 6, 5), ("B", 1, 0, 6, 5), ("C", 2, 0, 6, 5),
             ("D", 3, 0, 6, 5))
    _, missions = plan_missions(ps, CargoCfg(capacity=12, reload_s=60.0))
    assert len(missions) == 2
    assert all(m.load <= 12 for m in missions)
    # 第二架次出发时刻 = 第一架次飞行时间 + 60 s 装载
    assert missions[1].depart_s == pytest.approx(
        missions[0].flight_s + 60.0, abs=1e-6)


def test_route_order_is_nearest_neighbor():
    ps = pts(("Far", 300, 0, 2, 5), ("Near", 20, 0, 2, 5),
             ("Mid", 100, 0, 2, 5))
    order = route_order(ps, ["Far", "Near", "Mid"], (0.0, 0.0))
    assert order == ["Near", "Mid", "Far"]


def test_selection_deterministic():
    ps = pts(("A", 0, 0, 3, 4), ("B", 5, 5, 3, 4), ("C", 9, 9, 2, 5))
    assert select_points(ps, 12) == select_points(ps, 12)


def test_river_detour_inserted():
    wps = [(0.0, 0.0), (300.0, 300.0)]
    assert segment_crosses_band(wps[0], wps[1], 150.0, 180.0)
    out = detour_via_bridge(wps, 150.0, 180.0, bridge_x=260.0)
    assert len(out) == 4
    assert (260.0, 150.0) in out and (260.0, 180.0) in out
    # 绕行后的每一段都不再横穿禁飞带
    assert not any(segment_crosses_band(a, b, 150.0, 180.0)
                   for a, b in zip(out, out[1:]))
