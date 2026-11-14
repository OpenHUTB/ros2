# -*- coding: utf-8 -*-
"""MuJoCo 闭环冒烟：三个算例各真飞一次，落点误差与跟踪误差都要有上界。"""
import pytest

pytest.importorskip("mujoco")

from uav_drop_on_moving_target.core.cases import CASES, make_case   # noqa: E402
from uav_drop_on_moving_target.core.flyer import fly_case          # noqa: E402


@pytest.mark.parametrize("name", [c.name for c in CASES])
def test_closed_loop_release_hits_the_convoy(name):
    case = make_case(name)
    res = fly_case(case.track, case.vehicle, case.cfg)
    assert len(res.releases) == 1, "闭环没有在判定通过时松手"
    rec = res.releases[0]
    assert rec.error_m < 0.5, "落点误差 %.2f m 超过 0.5 m" % rec.error_m
    assert rec.t_fall == pytest.approx(case.cfg.t_fall, abs=0.05)


def test_tracking_error_stays_bounded():
    case = make_case("convoy_straight")
    res = fly_case(case.track, case.vehicle, case.cfg)
    assert res.max_track_error_m < 1.0
    assert res.max_error_release_m < 0.5
    assert res.final_error_m < 0.2


def test_closed_loop_matches_analytic_release_time():
    """闭环实际松手时刻应当与解析解一致（差 1 s 内）。"""
    from uav_drop_on_moving_target.core.release import solve_release
    case = make_case("convoy_straight")
    sol = solve_release(case.track, case.vehicle, case.cfg)
    res = fly_case(case.track, case.vehicle, case.cfg)
    assert abs(res.releases[0].t_release - sol.t_release) < 1.0
