# -*- coding: utf-8 -*-
"""弹道、车辆模型与释放时刻解算的单元测试。"""
import math

import numpy as np
import pytest

from uav_drop_on_moving_target.core.ballistics import (
    BallisticsCfg, fall_time, fall_time_from_state, impact_point, scatter_sigma,
    wind_drift)
from uav_drop_on_moving_target.core.cases import CASES, make_case
from uav_drop_on_moving_target.core.release import (
    DropCfg, evaluate_release, solve_release)
from uav_drop_on_moving_target.core.track import PolylineTrack
from uav_drop_on_moving_target.core.vehicle import Vehicle

CFG = DropCfg()


def test_fall_time_matches_analytic():
    assert fall_time(30.0) == pytest.approx(math.sqrt(2 * 30.0 / 9.81), rel=1e-12)
    assert fall_time(30.0) == pytest.approx(2.4729, abs=1e-3)


def test_fall_time_from_state_uses_vertical_speed():
    base = fall_time_from_state(30.0, 0.0)
    up = fall_time_from_state(30.0, 10.0)      # 抬头释放：落得更久
    down = fall_time_from_state(30.0, -5.0)
    assert base == pytest.approx(2.4729, abs=1e-3)
    assert up > base > down
    # z(t) 的正根：代回去应当恰好落地
    for vz in (-5.0, 0.0, 10.0):
        t = fall_time_from_state(30.0, vz)
        z = 30.0 + vz * t - 0.5 * 9.81 * t * t
        assert abs(z) < 1e-9


def test_wind_drift_is_linear_in_wind_and_time():
    t = CFG.t_fall
    d = wind_drift((6.0, -3.0), t)
    assert d[0] == pytest.approx(6.0 * t * 0.8, rel=1e-12)
    assert d[1] == pytest.approx(-3.0 * t * 0.8, rel=1e-12)


def test_impact_point_equals_manual_projection():
    p = np.array([100.0, 50.0, 30.0])
    v = np.array([10.0, 0.0, 0.0])
    imp = impact_point(p, v, (6.0, 0.0))
    t = CFG.t_fall
    assert imp[0] == pytest.approx(100.0 + 10.0 * t + 6.0 * t * 0.8, rel=1e-12)
    assert imp[1] == pytest.approx(50.0, abs=1e-12)


def test_impact_point_rejects_bad_states():
    with pytest.raises(ValueError):
        impact_point([1.0, 2.0], [0.0, 0.0, 0.0], (0.0, 0.0))
    with pytest.raises(ValueError):
        impact_point([1.0, 2.0, np.nan], [0.0, 0.0, 0.0], (0.0, 0.0))
    with pytest.raises(ValueError):
        impact_point([1.0, 2.0, 0.0], [0.0, 0.0, 0.0], (0.0, 0.0))


def test_scatter_sigma_scales_with_height():
    assert scatter_sigma(30.0) == pytest.approx(4.5)
    assert scatter_sigma(20.0) < scatter_sigma(30.0)


def test_drop_cfg_rejects_invalid_values():
    with pytest.raises(ValueError):
        DropCfg(alt_m=float("nan"))
    with pytest.raises(ValueError):
        DropCfg(alt_m=0.0)
    with pytest.raises(ValueError):
        DropCfg(wind=(1.0, 2.0, 3.0))
    with pytest.raises(ValueError):
        BallisticsCfg(g=0.0)


def test_solve_straight_convoy_hits_with_small_miss():
    case = make_case("convoy_straight")
    sol = solve_release(case.track, case.vehicle, case.cfg)
    assert sol.feasible
    assert sol.miss_m < 0.05
    # 释放点必须在无人机真正飞得到的航迹上
    s_release = float(np.linalg.norm(sol.release_xy - np.asarray(case.track.points[0])))
    assert s_release < case.track.length_m


def test_naive_baseline_equals_convoy_displacement():
    """瞄准车辆当前位置的误差 ≈ 车速 × 下落时间。"""
    case = make_case("convoy_straight")
    sol = solve_release(case.track, case.vehicle, case.cfg)
    expect = case.vehicle.speed * case.cfg.t_fall
    assert sol.naive_miss_m == pytest.approx(expect, rel=0.05)


def test_crosswind_shifts_release_earlier_by_half_drift():
    """风把落点往前推，释放点要提前；提前量 ≈ 风偏 / 相对速度。"""
    plain = make_case("convoy_straight")
    windy = make_case("convoy_crosswind")
    s0 = solve_release(plain.track, plain.vehicle, plain.cfg)
    s1 = solve_release(windy.track, windy.vehicle, windy.cfg)
    drift = float(np.linalg.norm(windy.cfg.drift))
    assert drift == pytest.approx(11.87, abs=0.05)
    relative = windy.track.speed - windy.vehicle.speed
    assert s0.t_release - s1.t_release == pytest.approx(drift / relative, abs=0.6)
    assert s1.miss_m < 0.05


def test_stopgo_uniform_assumption_is_badly_off():
    """忽略 6 s 停留的规划会偏出 30 m 以上，按真实轨迹解算则命中。"""
    case = make_case("convoy_stopgo")
    sol = solve_release(case.track, case.vehicle, case.cfg)
    assert sol.feasible and sol.miss_m < 0.05
    assert sol.uniform_miss_m > 30.0


def test_infeasible_when_track_is_too_short():
    veh = Vehicle("v", ((0.0, 0.0), (600.0, 0.0)), 6.0)
    track = PolylineTrack(((0.0, 0.0), (30.0, 0.0)), 10.0)   # 追不上
    sol = solve_release(track, veh, DropCfg(speed=10.0))
    assert not sol.feasible
    assert sol.advice


def test_evaluate_release_uses_real_state():
    case = make_case("convoy_straight")
    sol = solve_release(case.track, case.vehicle, case.cfg)
    p = np.array([sol.release_xy[0], sol.release_xy[1], case.cfg.alt_m])
    v = np.array([case.track.speed, 0.0, 0.0])
    ok = evaluate_release(p, v, sol.t_release, case.vehicle, case.cfg)
    assert ok["ok"] and ok["miss_m"] < 0.1
    # 竖直速度异常应当改变判定（下落时间变长，落点前移）
    bad = evaluate_release(p, np.array([case.track.speed, 0.0, 25.0]),
                           sol.t_release, case.vehicle, case.cfg)
    assert bad["miss_m"] > ok["miss_m"]
    # 非有限状态直接拒绝
    assert not evaluate_release([np.nan, 0.0, 30.0], v, 0.0,
                                case.vehicle, case.cfg)["ok"]
    # 低于地面的状态也拒绝（不是抛异常，而是给出可读原因）
    ground = evaluate_release([p[0], p[1], 0.0], v, sol.t_release,
                              case.vehicle, case.cfg)
    assert not ground["ok"] and "ground" in ground["reason"]


def test_vehicle_dwell_timeline():
    veh = Vehicle("v", ((0.0, 0.0), (60.0, 0.0), (120.0, 0.0)), 6.0,
                  dwell_s=((1, 6.0),))
    assert veh.arrival_time(1) == pytest.approx(10.0)
    assert veh.duration_s == pytest.approx(20.0 + 6.0)
    assert veh.pos_at(11.0)[0] == pytest.approx(60.0)      # 停留中
    assert veh.speed_at(11.0) == 0.0
    assert veh.pos_at(13.0)[0] == pytest.approx(60.0)      # 停留还没结束
    assert veh.pos_at(16.0)[0] == pytest.approx(60.0)      # 停留结束瞬间
    assert veh.pos_at(20.0)[0] == pytest.approx(84.0)      # 重新起步后


def test_track_starts_with_acceleration_ramp():
    tr = PolylineTrack(((0.0, 0.0), (100.0, 0.0)), 10.0, ramp_s=2.0)
    p0, v0 = tr.pos_vel(0.0)
    p1, v1 = tr.pos_vel(1.0)
    p2, v2 = tr.pos_vel(2.0)
    assert np.allclose(p0, 0.0) and np.allclose(v0, 0.0)
    assert v1[0] == pytest.approx(5.0)               # 匀加速到一半
    assert v2[0] == pytest.approx(10.0)
    assert p2[0] == pytest.approx(10.0)              # 0.5·a·t²
    assert tr.duration_s == pytest.approx(11.0)      # 10 s 巡航 + 1 s 加速


def test_cases_are_consistent():
    assert len(CASES) == 3
    for c in CASES:
        assert c.cfg.speed == pytest.approx(c.track.speed)
        assert c.cfg.alt_m > 0.0
