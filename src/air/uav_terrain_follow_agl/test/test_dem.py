# -*- coding: utf-8 -*-
"""DEM：网格插值、偏置、移位方向与误差统计。"""
import numpy as np
import pytest

from uav_terrain_follow_agl.core.dem import (DEGRADED_DEM, IDEAL_DEM, PRACTICAL_DEM,
                                             DemConfig, DigitalElevationModel)
from uav_terrain_follow_agl.core.terrain import Ridge, TerrainField, get_terrain


def test_ideal_dem_equals_truth():
    t = get_terrain("ridge_crossing")
    dem = DigitalElevationModel(t, IDEAL_DEM)
    for x in (0.0, 150.0, 280.0, 480.0, 599.0):
        assert dem.height(x, 0.0) == pytest.approx(t.elevation(x, 0.0), abs=1e-9)
        assert dem.error(x, 0.0) == pytest.approx(0.0, abs=1e-9)


def test_bias_shifts_uniformly_on_flat_ground():
    t = get_terrain("flat")
    dem = DigitalElevationModel(t, DemConfig(grid_m=0.0, bias_m=2.5))
    assert dem.height(50.0, 0.0) == pytest.approx(2.5)
    assert dem.error(50.0, 0.0) == pytest.approx(2.5)


def test_positive_shift_moves_features_forward():
    """shift_x > 0 表示 DEM 里的地形特征比实际提前出现。"""
    t = TerrainField(name="r", ridges=(Ridge(300.0, 0.0, 30.0, 20.0),))
    dem = DigitalElevationModel(t, DemConfig(grid_m=0.0, bias_m=0.0, shift_x_m=10.0))
    # DEM 在 x=290 读到的应是真值 x=280 的高程，即坡的「上游」
    assert dem.height(290.0, 0.0) == pytest.approx(t.elevation(280.0, 0.0))


def test_grid_interpolation_error_is_small_and_bounded():
    t = get_terrain("ridge_crossing")
    for grid in (1.0, 2.0, 4.0, 8.0):
        dem = DigitalElevationModel(t, DemConfig(grid_m=grid, bias_m=0.0, shift_x_m=0.0))
        xs = np.linspace(0.0, 700.0, 1401)
        err = np.abs(dem.error(xs, 0.0))
        # 线性插值的误差上界 ~ h''·g²/8，高斯地形上远小于 1 m
        assert err.max() < 0.05 * grid ** 2, f"grid={grid} err={err.max():.3f}"


def test_degraded_dem_is_worse_than_practical():
    t = get_terrain("mixed")
    xs = np.linspace(0.0, 700.0, 700)
    err_p = np.abs(DigitalElevationModel(t, PRACTICAL_DEM).error(xs, 0.0)).max()
    err_d = np.abs(DigitalElevationModel(t, DEGRADED_DEM).error(xs, 0.0)).max()
    assert err_d > err_p


def test_stats_reports_bias_dominated_error():
    t = get_terrain("gentle_slope")
    dem = DigitalElevationModel(t, DemConfig(grid_m=0.0, bias_m=1.2, shift_x_m=0.0))
    s = dem.stats(0.0, 600.0)
    assert s["mean_error_m"] == pytest.approx(1.2, abs=1e-6)
    assert s["rms_error_m"] == pytest.approx(1.2, abs=1e-6)


def test_scalar_and_array_query_agree():
    t = get_terrain("mixed")
    dem = DigitalElevationModel(t, PRACTICAL_DEM)
    xs = np.array([10.0, 200.0, 430.0])
    arr = dem.height(xs, 0.0)
    assert isinstance(arr, np.ndarray) and arr.shape == (3,)
    assert arr[2] == pytest.approx(dem.height(430.0, 0.0))


@pytest.mark.parametrize("bad", [dict(grid_m=-1.0), dict(bias_m=float("nan")),
                                 dict(shift_x_m=float("inf"))])
def test_dem_config_rejects_bad_values(bad):
    kwargs = dict(grid_m=2.0)
    kwargs.update(bad)
    with pytest.raises(ValueError):
        DemConfig(**kwargs)


def test_query_outside_grid_is_clamped_not_extrapolated():
    t = get_terrain("ridge_crossing")
    dem = DigitalElevationModel(t, PRACTICAL_DEM, x_range=(0.0, 100.0),
                                y_range=(-10.0, 10.0))
    # 超出网格后按边界值保持，不会外插成离谱数字
    assert abs(dem.height(500.0, 0.0)) < 1e3
