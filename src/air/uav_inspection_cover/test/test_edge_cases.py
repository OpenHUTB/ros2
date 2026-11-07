# -*- coding: utf-8 -*-
"""Edge cases: flyability, no-fly detour, coverage misses, degenerate inputs."""
import math

from uav_inspection_cover.core.checker import cover_report
from uav_inspection_cover.core.geometry import (Params, Tower, leg_flyable,
                                                seg_rect_dist)
from uav_inspection_cover.core.planner import (Corridor, build_cases,
                                               build_flyable_graph,
                                               dijkstra_path, make_route,
                                               nearest_neighbor, _connect,
                                               _replace_leg, route_cost,
                                               two_opt, progress_snapshot)

P = Params()
CASES = build_cases()


def _routes(name):
    c = CASES[name]
    nn = nearest_neighbor(c)
    return c, make_route(c, P, nn), make_route(c, P, two_opt(nn, route_cost(c, P)))


def test_final_route_fully_flyable():
    """改进前后最终航线的每一段都满足塔距/禁飞区约束。"""
    for name in CASES:
        c, base, imp = _routes(name)
        for route in (base, imp):
            wps = route.waypoints
            for i in range(1, len(wps)):
                assert leg_flyable(wps[i - 1], wps[i], c.towers, c.zones, P)


def test_river_route_detours_zone():
    c, base, imp = _routes("case_grid_river")
    # 绕行必产生额外航点；且没有任何一段进入河道禁飞区
    assert len(base.waypoints) > len(base.viewpoints) + 1
    for route in (base, imp):
        wps = route.waypoints
        for i in range(1, len(wps)):
            for z in c.zones:
                assert seg_rect_dist(wps[i - 1], wps[i], z) >= P.zone_margin_m


def test_leg_through_tower_not_flyable():
    tw = Tower("T01", 0.0, 0.0, 30.0)
    a, b = (-100.0, 5.0, 20.0), (100.0, 5.0, 20.0)
    assert not leg_flyable(a, b, [tw], (), P)          # 距塔仅 5 m
    a2, b2 = (-100.0, 40.0, 20.0), (100.0, 40.0, 20.0)
    assert leg_flyable(a2, b2, [tw], (), P)            # 距塔 40 m


def test_zone_blocks_leg():
    tw = Tower("T01", 0.0, 0.0, 30.0)
    zone = (-20.0, 90.0, 20.0, 130.0)
    assert not leg_flyable((-100.0, 110.0, 20.0), (100.0, 110.0, 20.0),
                           [tw], (zone,), P)
    assert leg_flyable((-100.0, 160.0, 20.0), (100.0, 160.0, 20.0),
                       [tw], (zone,), P)


def test_detour_keeps_endpoints():
    c = CASES["case_grid_river"]
    a, b = (-100.0, 110.0, 20.0), (1100.0, 110.0, 20.0)   # 直穿河道
    nodes, adj = build_flyable_graph(c, P)
    out = _replace_leg(a, b, c, P, nodes, adj)
    assert out[0] == a and out[-1] == b
    assert len(out) > 2                                    # 确实绕出了中间点
    for i in range(1, len(out)):
        for z in c.zones:
            assert seg_rect_dist(out[i - 1], out[i], z) >= P.zone_margin_m


def test_dijkstra_reaches_far_nodes():
    c, base, _ = _routes("case_grid_river")
    nodes, adj = build_flyable_graph(c, P)
    connected = [i for i in adj if adj[i]]
    s = min(connected, key=lambda i: nodes[i][0])
    g = max(connected, key=lambda i: nodes[i][0])
    path = dijkstra_path(nodes, adj, s, g)
    assert path[0] == s and path[-1] == g and len(path) >= 2


def test_cover_report_lists_missed():
    c, base, _ = _routes("case_grid_short")
    wps = base.waypoints
    # 把整条航线抬高 100 m：所有观察点都漏检
    lifted = [(x, y, z + 100.0) for x, y, z in wps]
    rep = cover_report(base.viewpoints, lifted, P.cover_radius_m)
    assert rep.rate == 0.0
    assert sorted(rep.missed) == sorted(vp.tag for vp in base.viewpoints)


def test_cover_radius_sensitivity():
    c, base, _ = _routes("case_grid_short")
    lifted = [(x, y, z + 100.0) for x, y, z in base.waypoints]
    rep_tight = cover_report(base.viewpoints, lifted, 15.0)
    rep_loose = cover_report(base.viewpoints, lifted, 150.0)
    assert rep_loose.rate >= rep_tight.rate
    assert rep_tight.missed and not rep_loose.missed


def test_single_tower_route():
    tw = Tower("T01", 500.0, 0.0, 40.0)
    c = Corridor("case_single", "single tower", (tw,), start=(0.0, 0.0))
    r = make_route(c, P)
    assert len(r.waypoints) >= 2 and len(r.viewpoints) == 3


def test_duplicate_free_waypoints():
    c, base, imp = _routes("case_grid_short")
    for route in (base, imp):
        wps = route.waypoints
        for i in range(1, len(wps)):
            assert math.dist(wps[i - 1], wps[i]) > 1e-6


def test_progress_snapshot_at_end_done():
    c, base, _ = _routes("case_grid_short")
    last = base.waypoints[-1]
    hint = len(base.waypoints) - 1          # 节点一路跟踪过来的单调推进
    st = progress_snapshot(base.waypoints, last, P.cover_radius_m, hint)
    assert st["done"] and st["progress"] == 100.0
