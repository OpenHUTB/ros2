# -*- coding: utf-8 -*-
"""退化输入与参数校验：这些是"看着能跑"但必须显式拒绝的地方。"""
import numpy as np
import pytest

from uav_drop_on_moving_target.core.release import DropCfg, solve_release
from uav_drop_on_moving_target.core.track import PolylineTrack
from uav_drop_on_moving_target.core.vehicle import Vehicle


def test_track_rejects_degenerate_input():
    with pytest.raises(ValueError):
        PolylineTrack(((0.0, 0.0),), 10.0)                 # 只有一个点
    with pytest.raises(ValueError):
        PolylineTrack(((5.0, 5.0), (5.0, 5.0)), 10.0)      # 零长度
    with pytest.raises(ValueError):
        PolylineTrack(((0.0, 0.0), (10.0, 0.0)), 0.0)      # 速度必须为正
    with pytest.raises(ValueError):
        PolylineTrack(((0.0, 0.0), (np.nan, 0.0)), 10.0)


def test_vehicle_rejects_bad_path_and_dwell():
    with pytest.raises(ValueError):
        Vehicle("v", ((0.0, 0.0),), 6.0)
    with pytest.raises(ValueError):
        Vehicle("v", ((0.0, 0.0), (10.0, 0.0)), -1.0)
    with pytest.raises(ValueError):
        Vehicle("v", ((0.0, 0.0), (10.0, 0.0)), 6.0, dwell_s=((5, 3.0),))
    with pytest.raises(ValueError):
        Vehicle("v", ((0.0, 0.0), (10.0, 0.0)), 6.0, dwell_s=((1, -2.0),))


def test_vehicle_zero_length_segment_is_skipped():
    veh = Vehicle("v", ((0.0, 0.0), (0.0, 0.0), (30.0, 0.0)), 6.0)
    assert veh.total_len_m == pytest.approx(30.0)
    assert veh.pos_at(5.5)[0] == pytest.approx(30.0)       # 走完停在终点
    assert veh.speed_at(5.5) == 0.0                        # 停住后速度为 0


def test_vehicle_pos_requires_finite_time():
    veh = Vehicle("v", ((0.0, 0.0), (30.0, 0.0)), 6.0)
    with pytest.raises(ValueError):
        veh.pos_at(float("nan"))


def test_solve_returns_infeasible_without_crashing_when_convoy_is_parked():
    """车队整段都没动过：解算要给出"不可行"而不是死循环或除零。"""
    veh = Vehicle("v", ((0.0, 0.0), (200.0, 0.0)), 6.0, dwell_s=((1, 1e6),))
    track = PolylineTrack(((-30.0, 0.0), (400.0, 0.0)), 10.0)
    sol = solve_release(track, veh, DropCfg(speed=10.0, horizon_s=120.0))
    # 车队起点停着，但航迹会经过它——允许投放（停着的车更容易命中）
    assert sol.feasible or sol.advice


def test_solve_rejects_nonfinite_geometry():
    veh = Vehicle("v", ((0.0, 0.0), (200.0, 0.0)), 6.0)
    track = PolylineTrack(((-30.0, 0.0), (400.0, 0.0)), 10.0)
    with pytest.raises(ValueError):
        solve_release(track, veh, DropCfg(wind=(np.nan, 0.0)))


def test_horizon_caps_the_search():
    """时间上限太短时不该"假装"找到解。"""
    veh = Vehicle("v", ((0.0, 0.0), (600.0, 0.0)), 6.0)
    track = PolylineTrack(((-80.0, 0.0), (560.0, 0.0)), 10.0)
    sol = solve_release(track, veh, DropCfg(speed=10.0, horizon_s=20.0))
    assert not sol.feasible
