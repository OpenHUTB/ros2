# -*- coding: utf-8 -*-
"""在线节点状态机：不 import rclpy 也能测。"""
import json
import math

import numpy as np
import pytest

from uav_terrain_follow_agl.ros.terrain_follow_node import (DEFAULT_PARAMS, FollowState,
                                                           node_params)


def _flat_profile(n=76, height=0.0, step=2.0):
    return [height + 0.0 * i for i in range(n)]


def _state(**over):
    st = FollowState(over)
    st.on_odom(0.0, 15.0, 8.0)
    st.on_agl(15.0)
    return st


def test_default_params_are_valid():
    p = node_params()
    assert p["strategy"] == "dem_preview"
    assert set(DEFAULT_PARAMS) <= set(p)


@pytest.mark.parametrize("bad", [dict(strategy="mpc"), dict(agl_set_m=0.0),
                                 dict(agl_set_m=200.0), dict(climb_rate_max_mps=0.0),
                                 dict(profile_len=1), dict(profile_step_m=0.0),
                                 dict(control_dt=0.0)])
def test_node_params_reject_bad_values(bad):
    with pytest.raises(ValueError):
        node_params(bad)


def test_profile_hook_strategy_for_all_modes():
    for name in ("dem_preview", "reactive_agl", "constant_msl"):
        st = _state(strategy=name)
        st.on_profile(_flat_profile())
        z, vz = st.step()
        assert math.isfinite(z) and math.isfinite(vz)
        assert st.diag()["strategy"] == name


def test_step_requires_odom_first():
    st = FollowState()
    with pytest.raises(ValueError):
        st.step()


def test_dem_preview_waits_for_profile():
    st = _state()
    z, vz = st.step()
    assert (z, vz) == (15.0, 0.0)
    assert st.diag()["profile_updates"] == 0
    assert st.diag()["ready"] is True


def test_dem_preview_raises_setpoint_when_ridge_ahead():
    st = _state()
    # 前方 60 m 处起一道 25 m 高的地形（剖面步长 2 m）
    profile = [0.0] * 30 + [25.0] * 46
    st.on_profile(profile)
    z = vz = 0.0
    vz_peak = 0.0
    for _ in range(200):                   # 2 s，够爬升率积起来
        z, vz = st.step()
        vz_peak = max(vz_peak, vz)
    assert z > 15.0 + 4.0                  # 还没到坡就已经爬到 19 m 以上
    assert vz_peak > 1.0                   # 中途确实按爬升率上限在爬


def test_reactive_raises_setpoint_on_low_agl():
    st = _state(strategy="reactive_agl")
    for _ in range(20):
        st.on_agl(8.0)                     # 低了 7 m
        z, vz = st.step()
    assert vz > 0.0
    assert st.diag()["error_m"] == pytest.approx(7.0, abs=0.1)


def test_constant_msl_ignores_measurements():
    st = _state(strategy="constant_msl")
    for _ in range(30):
        st.on_agl(3.0)
        st.on_profile([40.0] * 76)
        z, vz = st.step()
    assert z == pytest.approx(15.0)
    assert vz == 0.0


def test_profile_validation():
    st = FollowState()
    with pytest.raises(ValueError):
        st.on_profile([1.0])
    with pytest.raises(ValueError):
        st.on_profile([1.0, float("nan")])
    with pytest.raises(ValueError):
        st.on_profile([[1.0, 2.0], [3.0, 4.0]])


def test_agl_validation():
    st = FollowState()
    with pytest.raises(ValueError):
        st.on_agl(-1.0)
    with pytest.raises(ValueError):
        st.on_agl(float("inf"))


def test_odom_validation():
    st = FollowState()
    with pytest.raises(ValueError):
        st.on_odom(float("nan"), 15.0, 1.0)


def test_diag_is_json_serialisable_and_has_counts():
    st = _state()
    st.on_profile(_flat_profile())
    st.step()
    d = st.diag()
    text = json.dumps(d, ensure_ascii=False)
    assert "trim_m" in d and "z_ref_m" in d
    assert d["agl_samples"] == 1 and d["profile_updates"] == 1
    assert "15" in text or "agl_set_m" in text


def test_dem_preview_trim_moves_toward_bias():
    st = _state(trim_ki=0.2)               # 加速积分便于观察方向
    st.on_profile(_flat_profile(height=2.0))   # DEM 偏高 2 m
    z_prev = None
    for _ in range(200):
        z, _ = st.step()
        st.on_odom(0.0, z, 0.0)
        st.on_agl(z)                        # 平地：AGL 即高度
        z_prev = z
    assert z_prev is not None
    assert st.diag()["trim_m"] < 0.0        # 往「压回设定高度」的方向走
