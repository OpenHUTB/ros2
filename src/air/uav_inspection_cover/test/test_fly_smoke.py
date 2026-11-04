# -*- coding: utf-8 -*-
"""MuJoCo closed-loop smoke tests (small scale, < 10 s wall each)."""
import time

import pytest

from uav_inspection_cover.core.flyer import build_timeline, fly_legs
from uav_inspection_cover.core.geometry import Params
from uav_inspection_cover.core.planner import (build_cases, make_route,
                                               nearest_neighbor)

P = Params()


def _short_route():
    c = build_cases()["case_grid_short"]
    return make_route(c, P, nearest_neighbor(c))


@pytest.fixture(scope="module")
def stats():
    route = _short_route()
    t0 = time.time()
    st = fly_legs(route.waypoints, P, legs=2, hover_at=route.hover_at)
    return st, time.time() - t0


def test_fly_smoke_two_legs(stats):
    st, wall = stats
    assert st.legs == 2
    assert st.planned_m > 100.0
    assert st.actual_m > 0.0


def test_fly_smoke_wall_time(stats):
    st, wall = stats
    assert wall < 10.0                                    # 单次冒烟 < 10 s


def test_fly_smoke_tracks_reference(stats):
    st, wall = stats
    # 转角减速 + 末端保持后：实际航程接近计划，终点收敛
    assert abs(st.actual_m - st.planned_m) / st.planned_m < 0.10
    assert st.arrival_error_m < 1.0


def test_timeline_hover_extends_by_hover_time():
    route = _short_route()
    path = route.waypoints[:3]
    hover = {i for i in route.hover_at if 0 < i < 3}
    arrive, leave = build_timeline(path, hover, P)
    for i in hover:
        assert leave[i] - arrive[i] == pytest.approx(P.hover_time)
    for i in range(len(path)):
        if i not in hover:
            assert leave[i] == pytest.approx(arrive[i])
