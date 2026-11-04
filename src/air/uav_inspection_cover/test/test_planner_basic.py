# -*- coding: utf-8 -*-
"""Planner basic behavior: observe points, NN + 2-opt, route assembly."""
import math

import pytest

from uav_inspection_cover.core.checker import cover_report
from uav_inspection_cover.core.geometry import Params, Tower, \
    corridor_viewpoints, n_angles, point_seg_dist
from uav_inspection_cover.core.planner import (build_cases, make_route,
                                               nearest_neighbor, order_cost,
                                               progress_snapshot, route_cost,
                                               sort_viewpoints, two_opt)

P = Params()
CASES = build_cases()


def _routes(name):
    c = CASES[name]
    nn = nearest_neighbor(c)
    base = make_route(c, P, nn)
    imp_order = two_opt(nn, route_cost(c, P))
    imp = make_route(c, P, imp_order)
    return c, nn, imp_order, base, imp


def test_observe_point_geometry():
    tw = Tower("T01", 100.0, 200.0, 40.0)
    pts = corridor_viewpoints([tw], P)
    assert len(pts) == 3                       # 高塔 3 个角度
    for vp in pts:
        assert math.hypot(vp.x - 100.0, vp.y - 200.0) == pytest.approx(P.offset_m)
        assert vp.z == min(max(P.height_ratio * 40.0, P.min_height_m),
                           P.max_height_m)


def test_observe_point_count_2_to_3():
    c = CASES["case_grid_short"]
    vps = corridor_viewpoints(c.towers, P)
    by_tower = {}
    for vp in vps:
        by_tower.setdefault(vp.tower, 0)
        by_tower[vp.tower] += 1
    assert all(2 <= n <= 3 for n in by_tower.values())
    assert len(vps) == sum(n_angles(t) for t in c.towers)


def test_point_seg_dist_basics():
    assert point_seg_dist(0, 5, -10, 0, 10, 0) == 5.0     # 内垂足
    assert point_seg_dist(15, 0, -10, 0, 10, 0) == 5.0    # 端点外
    assert point_seg_dist(0, 0, 3, 4, 3, 4) == 5.0        # 退化为点


def test_nn_order_is_permutation():
    for name in CASES:
        c = CASES[name]
        assert sorted(nearest_neighbor(c)) == list(range(len(c.towers)))


def test_two_opt_never_worsens():
    for name in CASES:
        c, nn, imp, base, imp_route = _routes(name)
        assert imp_route.distance_m <= base.distance_m + 1e-6


def test_two_opt_improves_all_cases():
    for name in CASES:
        c, nn, imp, base, imp_route = _routes(name)
        assert base.distance_m - imp_route.distance_m > 50.0


def test_two_opt_only_accepts_decrease():
    """2-opt 中间过程只接受严格下降：最终序代价 ≤ 初始序代价。"""
    c = CASES["case_grid_short"]
    cost = route_cost(c, P)
    nn = nearest_neighbor(c)
    imp = two_opt(nn, cost)
    assert cost(imp) < cost(nn) - 1e-6


def test_route_visits_all_viewpoints():
    for name in CASES:
        c, nn, imp, base, imp_route = _routes(name)
        vps = corridor_viewpoints(c.towers, P)
        assert len(base.viewpoints) == len(vps) == len(imp_route.viewpoints)
        wp_set = {(round(x, 3), round(y, 3))
                  for x, y, _ in base.waypoints}
        for vp in vps:
            assert (round(vp.x, 3), round(vp.y, 3)) in wp_set


def test_route_starts_at_corridor_start():
    c, nn, imp, base, imp_route = _routes("case_grid_short")
    assert base.waypoints[0][:2] == c.start
    assert base.waypoints[0][2] == P.min_height_m


def test_duration_formula():
    c, nn, imp, base, imp_route = _routes("case_grid_short")
    expect = base.distance_m / P.cruise_speed + P.hover_time * len(base.viewpoints)
    assert abs(base.duration_s(P) - expect) < 1e-6


def test_arrival_angle_sort_rotational():
    tw = Tower("T01", 0.0, 0.0, 40.0)
    pts = corridor_viewpoints([tw], P)
    seq = sort_viewpoints(pts, tw, (-50.0, 0.0, 20.0))
    assert {vp.tag for vp in seq} == {vp.tag for vp in pts}
    again = sort_viewpoints(pts, tw, (-50.0, 0.0, 20.0))
    assert [vp.tag for vp in seq] == [vp.tag for vp in again]   # 确定性


def test_progress_snapshot_marches_forward():
    c, nn, imp, base, _ = _routes("case_grid_short")
    wps = base.waypoints
    s0 = progress_snapshot(wps, (1e6, 1e6, 0), P.cover_radius_m)
    assert s0["target_index"] == 0 and s0["progress"] == 0.0
    hint = 0
    for i, w in enumerate(wps):
        st = progress_snapshot(wps, w, P.cover_radius_m, hint)
        hint = st["target_index"]
    assert st["done"] and st["progress"] == 100.0


def test_coverage_full_on_own_route():
    c, nn, imp, base, imp_route = _routes("case_grid_short")
    rep = cover_report(base.viewpoints, base.waypoints, P.cover_radius_m)
    assert rep.rate == 1.0 and rep.missed == []
