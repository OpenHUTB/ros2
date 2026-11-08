#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo closed-loop smoke test: small route, 2 drops, must stay stable."""
import numpy as np
import pytest

from uav_emergency_drop.core.ballistics import make_release_plan
import uav_emergency_drop.core.flyer as flyer_mod
from uav_emergency_drop.core.flyer import (BRIDGE_CHANNEL_HALF_M,
                                           _build_legs, build_route,
                                           bridge_channel_deviation,
                                           fly_drop_route)
from uav_emergency_drop.core.cases import (CASE_AID_RIVER, CASE_AID_VILLAGE,
                                           CASE_AID_WIND)

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def smoke_flight():
    """两投放点小航线，控制测试时长（<10 s 墙钟量级）。"""
    case = CASE_AID_VILLAGE
    targets = [case.points[0], case.points[1]]
    route = [case.cargo.depot] + [(p.x, p.y) for p in targets]
    true_targets = [p.xy for p in targets]
    plan = make_release_plan(20.0, 6.0, (0.0, 0.0))
    result = fly_drop_route(route, true_targets, plan, dt=0.01,
                            record_every=5)
    return plan, result


def test_smoke_flight_completes_with_releases(smoke_flight):
    plan, result = smoke_flight
    assert len(result.releases) == 2
    assert len(result.times) > 100
    assert result.pos.shape == (len(result.times), 3)
    assert result.wall_s > 0.0


def test_tracking_error_below_5m(smoke_flight):
    _plan, result = smoke_flight
    # 规范要求：投放时（窗口附近）跟踪误差 < 5 m
    assert result.max_track_error_release_m < 5.0, (
        "release-window tracking error %.2f m"
        % result.max_track_error_release_m)
    # 末端悬停稳定：最后 0.8 s 平均跟踪误差 < 2 m
    assert result.final_hover_error_m < 2.0, (
        "final hover error %.2f m" % result.final_hover_error_m)


def test_impact_error_bounded(smoke_flight):
    plan, result = smoke_flight
    for rec in result.releases:
        # 无风下解析落点应接近目标（跟踪误差 + 前抛时间放大）
        assert rec.error_m < 6.0, (
            "impact error %.2f m at t=%.2f" % (rec.error_m, rec.t_release))


def test_wind_case_route_is_upwind_shifted():
    route, targets, _idx, _nc = build_route(CASE_AID_WIND)
    plan = CASE_AID_WIND.plan
    # 越顶点 = 真实目标 - drift（逆风偏移）
    for wp, tgt in zip(route[1:], targets):
        assert wp[0] == pytest.approx(float(tgt[0] - plan.drift[0]), abs=1e-6)
        assert wp[1] == pytest.approx(float(tgt[1] - plan.drift[1]), abs=1e-6)


def test_wind_case_smoke_flight_hits_target():
    """侧风 case 闭环：风补偿航线 + 弹道风偏应把落点拉回真实目标附近。"""
    case = CASE_AID_WIND
    route, targets, idx, no_cut = build_route(case)
    result = fly_drop_route(route[:idx[0] + 1], targets[:1], case.plan,
                            target_idx=[idx[0]], dt=0.01, record_every=5)
    assert len(result.releases) == 1
    assert result.releases[0].error_m < 6.0


def test_river_route_goes_through_bridge():
    """河流 case 闭环航线必须经桥梁通道横穿禁飞带。"""
    case = CASE_AID_RIVER
    route, targets, idx, no_cut = build_route(case)
    y_lo, y_hi = case.river
    # 每段都不得横穿禁飞带
    for a, b in zip(route, route[1:]):
        assert not ((a[1] < y_lo and b[1] > y_hi) or
                    (a[1] > y_hi and b[1] < y_lo))
    # 绕行使航线总长大于直线总长
    detour_len = sum(float(np.linalg.norm(np.array(b) - np.array(a)))
                     for a, b in zip(route, route[1:]))
    wps = [np.asarray(case.cargo.depot, dtype=float)] + \
        [np.asarray(t, dtype=float) for t in targets]
    straight = sum(float(np.linalg.norm(wps[i + 1] - wps[i]))
                   for i in range(len(wps) - 1))
    assert detour_len > straight


def test_releases_fire_through_evaluate_drop():
    """闭环触发与 ROS 节点共用 evaluate_drop（此前按计划位置触发，
    节点真正用的判据链路没被闭环覆盖）。"""
    case = CASE_AID_WIND
    route, targets, idx, no_cut = build_route(case)
    calls = {"n": 0}
    real = flyer_mod.evaluate_drop

    def counting(*a, **k):
        calls["n"] += 1
        return real(*a, **k)

    flyer_mod.evaluate_drop = counting
    try:
        result = fly_drop_route(route[:idx[0] + 1], targets[:1], case.plan,
                                target_idx=[idx[0]], dt=0.01, record_every=5)
    finally:
        flyer_mod.evaluate_drop = real
    assert calls["n"] > 100          # 每个控制周期都过一遍判定
    assert len(result.releases) == 1


def test_river_channel_reference_and_flight():
    """过河段的参考轨迹与实际轨迹都必须落在桥通道半宽内。"""
    case = CASE_AID_RIVER
    route, targets, idx, no_cut = build_route(case)
    legs, _dl, _ds = _build_legs(route, case.drop_h,
                                 case.plan.lead_m, no_cut=no_cut)
    ref_dev = bridge_channel_deviation(legs, case.river, case.bridge_x)
    assert ref_dev <= BRIDGE_CHANNEL_HALF_M

    result = fly_drop_route(route, targets, case.plan,
                            target_idx=idx, dt=0.01, record_every=5,
                            no_cut=no_cut)
    assert len(result.releases) == 3
    y_lo, y_hi = case.river
    in_band = result.pos[(result.pos[:, 1] >= y_lo)
                         & (result.pos[:, 1] <= y_hi)]
    assert len(in_band) > 0
    act_dev = float(np.max(np.abs(in_band[:, 0] - case.bridge_x)))
    assert act_dev <= BRIDGE_CHANNEL_HALF_M + 1e-6
