# -*- coding: utf-8 -*-
"""地形场：解析值、梯度、坡度/爬升率换算与参数校验。"""
import math

import numpy as np
import pytest

from uav_terrain_follow_agl.core.terrain import (PRESETS, Ridge, TerrainField,
                                                get_terrain, list_terrains)


def test_flat_is_zero_everywhere():
    t = get_terrain("flat")
    assert t.elevation(0.0, 0.0) == 0.0
    assert t.elevation(123.4, -56.7) == 0.0


def test_slope_is_tan_of_angle():
    t = TerrainField(name="s", slope_deg=10.0)
    assert t.elevation(100.0, 0.0) == pytest.approx(100.0 * math.tan(math.radians(10.0)))
    dhdx, dhdy = t.gradient(50.0, 3.0)
    assert dhdx == pytest.approx(math.tan(math.radians(10.0)))
    assert dhdy == pytest.approx(0.0)


def test_ridge_peak_at_centre():
    t = TerrainField(name="r", ridges=(Ridge(100.0, 0.0, 20.0, 10.0),))
    assert t.elevation(100.0, 0.0) == pytest.approx(20.0)
    assert t.elevation(100.0 + 10.0 * math.sqrt(2.0) * 0.5, 0.0) == pytest.approx(
        20.0 * math.exp(-0.25))


def test_gradient_matches_finite_difference():
    t = get_terrain("mixed")
    for x in (120.0, 245.0, 430.0, 560.0):
        for y in (-5.0, 0.0, 7.0):
            dhdx, dhdy = t.gradient(x, y)
            h = 1e-4
            fd_x = (t.elevation(x + h, y) - t.elevation(x - h, y)) / (2 * h)
            fd_y = (t.elevation(x, y + h) - t.elevation(x, y - h)) / (2 * h)
            assert dhdx == pytest.approx(fd_x, rel=1e-4, abs=1e-5)
            assert dhdy == pytest.approx(fd_y, rel=1e-4, abs=1e-5)


def test_array_and_scalar_agree():
    t = get_terrain("ridge_crossing")
    xs = np.array([0.0, 100.0, 280.0])
    arr = t.elevation(xs, 0.0)
    assert isinstance(arr, np.ndarray)
    assert arr.shape == (3,)
    assert arr[2] == pytest.approx(t.elevation(280.0, 0.0))
    # 二维查询（y 变化）也应保持 (ny, nx) 形状
    grid = t.elevation(xs[None, :], np.array([-10.0, 10.0])[:, None])
    assert grid.shape == (2, 3)


def test_max_slope_picks_the_steepest_ridge_side():
    t = get_terrain("ridge_crossing")
    got = t.max_slope(0.0, 600.0)
    # 高斯隆起的最陡坡度 = |A|/(σ√e)
    expect = 30.0 / (25.0 * math.sqrt(math.e))
    assert got == pytest.approx(expect, rel=2e-2)


def test_ridge_peak_slope_matches_analytic():
    r = Ridge(0.0, 0.0, 30.0, 25.0)
    assert r.peak_slope() == pytest.approx(30.0 / (25.0 * math.sqrt(math.e)))


def test_required_climb_rate_scales_with_speed():
    t = get_terrain("ridge_crossing")
    r4 = t.required_climb_rate(4.0, 0.0, 600.0)
    r8 = t.required_climb_rate(8.0, 0.0, 600.0)
    assert r8 == pytest.approx(2.0 * r4)
    assert t.max_speed_for_climb(r8, 0.0, 600.0) == pytest.approx(8.0)


def test_max_speed_for_climb_on_flat_is_infinite():
    t = get_terrain("flat")
    assert t.max_speed_for_climb(3.0, 0.0, 100.0) == float("inf")


def test_clearance_sign():
    t = get_terrain("ridge_crossing")
    assert t.clearance(0.0, 20.0) == pytest.approx(20.0)
    assert t.clearance(280.0, 20.0) < 0.0


def test_terrain_rate_uses_gradient():
    t = TerrainField(name="s", slope_deg=10.0)
    s = math.tan(math.radians(10.0))
    assert t.terrain_rate(0.0, 8.0) == pytest.approx(8.0 * s)


@pytest.mark.parametrize("bad", [dict(sigma=0.0), dict(sigma=-1.0), dict(amplitude=0.0)])
def test_ridge_rejects_bad_params(bad):
    kwargs = dict(x=0.0, y=0.0, amplitude=1.0, sigma=1.0)
    kwargs.update(bad)
    with pytest.raises(ValueError):
        Ridge(**kwargs)


def test_terrain_rejects_bad_slope():
    with pytest.raises(ValueError):
        TerrainField(name="bad", slope_deg=90.0)


def test_unknown_terrain_lists_options():
    with pytest.raises(KeyError) as exc:
        get_terrain("no_such")
    assert "ridge_crossing" in str(exc.value)


def test_presets_are_callable_and_listed():
    assert set(list_terrains()) == set(PRESETS)
    for name in list_terrains():
        assert isinstance(PRESETS[name](), TerrainField)


def test_profile_length_and_monotone_on_slope():
    t = TerrainField(name="s", slope_deg=5.0)
    xs, hs = t.profile(0.0, 100.0, n=11)
    assert xs.shape == hs.shape == (11,)
    assert np.all(np.diff(hs) > 0.0)
    with pytest.raises(ValueError):
        t.profile(0.0, 10.0, n=1)


def test_summary_has_expected_keys():
    s = get_terrain("mixed").summary()
    assert set(s) >= {"name", "base_m", "slope_deg", "ridges",
                      "min_elevation_m", "max_elevation_m", "max_slope_deg"}
    assert s["max_elevation_m"] > s["min_elevation_m"]
