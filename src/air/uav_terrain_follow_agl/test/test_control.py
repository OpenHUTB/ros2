# -*- coding: utf-8 -*-
"""三种策略：定高不变、纯反馈的解析滞后、DEM 前馈的提前量与限幅。"""
import math

import numpy as np
import pytest

from uav_terrain_follow_agl.core.control import (ConstantMSL, DemPreview,
                                                ReactiveAGL, VerticalLimits,
                                                make_strategy, steady_state_lag_m)
from uav_terrain_follow_agl.core.dem import (IDEAL_DEM, PRACTICAL_DEM, DemConfig,
                                             DigitalElevationModel)
from uav_terrain_follow_agl.core.terrain import Ridge, TerrainField, get_terrain

DT = 0.01


def _dem(terrain, cfg=IDEAL_DEM):
    return DigitalElevationModel(terrain, cfg)


# --------------------------------------------------------------------------- #
# 公共
# --------------------------------------------------------------------------- #
def test_limits_clamp_rate_and_accel():
    lim = VerticalLimits(climb_rate_mps=3.0, climb_accel_mps2=2.0)
    v = 0.0
    for _ in range(10):                       # 0.1 s 内最多加速到 0.2 m/s
        v = lim.limit(99.0, v, DT)
    assert v == pytest.approx(0.2)
    v = 0.0
    for _ in range(1000):
        v = lim.limit(99.0, v, DT)
    assert v == pytest.approx(3.0)


@pytest.mark.parametrize("bad", [dict(climb_rate_mps=0.0), dict(climb_accel_mps2=-1.0)])
def test_limits_reject_bad_values(bad):
    with pytest.raises(ValueError):
        VerticalLimits(**bad)


def test_make_strategy_dispatch_and_unknown():
    assert isinstance(make_strategy("constant_msl"), ConstantMSL)
    assert isinstance(make_strategy("reactive_agl"), ReactiveAGL)
    assert isinstance(make_strategy("dem_preview"), DemPreview)
    with pytest.raises(KeyError):
        make_strategy("mpc")


def test_steady_state_lag_formula():
    s = math.tan(math.radians(6.0))
    assert steady_state_lag_m(10.0, s, kp=1.2, latency_s=0.17) == pytest.approx(
        10.0 * s * (1.0 / 1.2 + 0.17))


# --------------------------------------------------------------------------- #
# 定高
# --------------------------------------------------------------------------- #
def test_constant_msl_never_changes():
    strat = ConstantMSL()
    strat.reset(get_terrain("ridge_crossing"), _dem(get_terrain("ridge_crossing")),
                0.0, 15.0, 15.0)
    for i in range(200):
        z, vz = strat.step(i * DT, DT, 50.0 + i * 0.08, 15.0, 8.0, 15.0 - 10.0)
        assert z == pytest.approx(15.0)
        assert vz == 0.0


# --------------------------------------------------------------------------- #
# 纯反馈
# --------------------------------------------------------------------------- #
def _ramp_loop(strategy, speed=10.0, slope=math.tan(math.radians(6.0)),
               n=4000, agl_set=15.0):
    """恒坡上的闭环：把策略自己的 z 当被控对象，喂回真实 AGL。"""
    terrain = TerrainField(name="ramp", slope_deg=math.degrees(math.atan(slope)))
    strategy.reset(terrain, _dem(terrain), 0.0, agl_set, agl_set)
    x, z = 0.0, agl_set
    err = []
    for i in range(n):
        agl_meas = z - terrain.elevation(x, 0.0)
        z, vz = strategy.step(i * DT, DT, x, z, speed, agl_meas)
        x += speed * DT
        err.append(agl_set - (z - terrain.elevation(x, 0.0)))
    return np.asarray(err)


def test_proportional_loop_matches_analytic_lag():
    """纯比例（ki=0）在恒坡上的稳态滞后 = v·tanθ/Kp。

    这条环里没有传感器延迟（AGL 立刻可用），所以核对的是解析式的比例项；
    带 0.12 s 传输延迟的完整版本在 MuJoCo 闭环里另有一组数（见文档第 3 节）。
    """
    speed, slope = 10.0, math.tan(math.radians(6.0))
    strat = ReactiveAGL(VerticalLimits(), kp=1.2, ki=0.0)
    err = _ramp_loop(strat, speed=speed, slope=slope)
    measured = float(np.mean(err[-500:]))
    theory = steady_state_lag_m(speed, slope, 1.2, 0.0)
    assert measured == pytest.approx(theory, rel=0.12)


def test_integral_removes_steady_state_lag():
    speed, slope = 10.0, math.tan(math.radians(6.0))
    strat = ReactiveAGL(VerticalLimits(), kp=1.2, ki=0.25)
    err = _ramp_loop(strat, speed=speed, slope=slope)
    assert abs(float(np.mean(err[-500:]))) < 0.15


def test_reactive_is_silent_before_first_sample():
    strat = ReactiveAGL()
    terrain = get_terrain("flat")
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    z, vz = strat.step(0.0, DT, 0.0, 15.0, 8.0, None)
    assert (z, vz) == (15.0, 0.0)


def test_reactive_respects_climb_rate_limit():
    strat = ReactiveAGL(VerticalLimits(climb_rate_mps=3.0, climb_accel_mps2=2.0))
    terrain = get_terrain("flat")
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    for i in range(500):
        _, vz = strat.step(i * DT, DT, 0.0, 15.0, 0.0, 1.0)   # 误差 14 m
        assert abs(vz) <= 3.0 + 1e-9


# --------------------------------------------------------------------------- #
# DEM 前馈
# --------------------------------------------------------------------------- #
def test_preview_is_flat_on_flat_terrain():
    terrain = get_terrain("flat")
    strat = DemPreview()
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    for i in range(200):
        z, vz = strat.step(i * DT, DT, 100.0, 15.0, 8.0, 15.0)
        assert abs(z - 15.0) < 0.05
        assert abs(vz) < 0.05


def test_preview_climbs_before_the_terrain_rises():
    """山脊还在 80 m 外就该开始爬——这是纯反馈做不到的。"""
    terrain = get_terrain("ridge_crossing")
    strat = DemPreview(VerticalLimits(climb_rate_mps=3.0))
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    speed = 8.0
    x = 200.0                                  # 山脊在 280 m，地形此处基本为 0
    assert terrain.elevation(x, 0.0) < 0.3
    z_ref = strat.preview_reference(x, speed)
    assert z_ref > 15.0 + 1.0


def test_preview_lead_distance_matches_climb_budget():
    """提前量 ≈ A/ṙ·v：30 m 山脊、3 m/s、8 m/s → 80 m。"""
    terrain = get_terrain("ridge_crossing")
    strat = DemPreview(VerticalLimits(climb_rate_mps=3.0))
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    speed = 8.0
    # 找到「参考刚开始超过设定高度 0.2 m」的位置
    start = None
    for x in np.arange(120.0, 280.0, 0.5):
        if strat.preview_reference(float(x), speed) > 15.2:
            start = float(x)
            break
    assert start is not None
    lead = 280.0 - start
    assert 70.0 < lead < 95.0


def test_preview_respects_climb_rate_limit():
    terrain = get_terrain("ridge_crossing")
    strat = DemPreview(VerticalLimits(climb_rate_mps=3.0, climb_accel_mps2=2.0))
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    for i in range(2000):
        _, vz = strat.step(i * DT, DT, float(i * 0.08), 15.0, 8.0, 15.0)
        assert abs(vz) <= 3.0 + 1e-9


def test_preview_trim_absorbs_dem_bias():
    """DEM 偏高 2 m 且地形平坦时，trim 应慢慢收敛到 −2 m。"""
    terrain = get_terrain("flat")
    dem = _dem(terrain, DemConfig(grid_m=0.0, bias_m=2.0))
    strat = DemPreview(ki=0.05, trim_limit=10.0)
    strat.reset(terrain, dem, 0.0, 17.0, 15.0)
    z = 17.0
    for i in range(6000):                      # 60 s
        agl_meas = z                                # 平地：AGL 就是高度
        z, _ = strat.step(i * DT, DT, 0.0, z, 0.0, agl_meas)
    assert strat.trim == pytest.approx(-2.0, abs=0.4)


def test_preview_reference_from_profile_validates_shapes():
    terrain = get_terrain("flat")
    strat = DemPreview()
    strat.reset(terrain, _dem(terrain), 0.0, 15.0, 15.0)
    with pytest.raises(ValueError):
        strat.reference_from_profile([1.0, 2.0], [0.0])
    with pytest.raises(ValueError):
        strat.reference_from_profile([], [])


def test_preview_profile_hook_replaces_dem():
    heights = [0.0, 0.0, 30.0]
    strat = DemPreview(limits=VerticalLimits(climb_rate_mps=3.0),
                       profile_hook=lambda: heights, horizon_m=4.0, step_m=2.0)
    strat.reset(None, None, 0.0, 15.0, 15.0)
    # 30 m 地形在 4 m 外、8 m/s 时只剩 0.5 s 提前量 → 远高于 45 m
    assert strat.preview_reference(0.0, 8.0) > 30.0


def test_preview_without_dem_or_hook_raises():
    with pytest.raises(ValueError):
        DemPreview().reset(get_terrain("flat"), None, 0.0, 15.0, 15.0)
