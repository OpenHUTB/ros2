# -*- coding: utf-8 -*-
"""精度评估：解析预期要对得上蒙特卡洛，几何结论要站得住。"""
import math

import numpy as np
import pytest

from uav_target_geolocation.core.analysis import (QUAD_MASS_KG, compare_to_theory,
                                                 from_station_directions,
                                                 monte_carlo, sweep_angle,
                                                 theory_single_cep50,
                                                 wind_tilt_deg)
from uav_target_geolocation.core.cases import (CASES, GeoCase, Station, get_case,
                                               sweep_case)
from uav_target_geolocation.core.geometry import los_to


def test_wind_tilt_matches_tangent_of_force_ratio():
    f = 1.0
    expect = math.degrees(math.atan2(f, QUAD_MASS_KG * 9.81))
    assert wind_tilt_deg((f, 0.0)) == pytest.approx(expect, rel=1e-9)
    assert wind_tilt_deg((0.0, 0.0)) == pytest.approx(0.0)


def test_theory_matches_monte_carlo():
    """理论 CEP50 与打点结果的比值应接近 1，差太多说明公式或实现有一边错了。"""
    case = get_case("single_station")
    res = compare_to_theory(case, n=400, samples_per_station=120)
    assert 0.6 < res["ratio"] < 1.8


def test_theory_single_cep50_scales_with_bias():
    case = get_case("single_station")
    small = theory_single_cep50(case)
    big = theory_single_cep50(GeoCase(name="x", title="x",
                                      stations=case.stations, bias_deg=0.4))
    assert big > 1.5 * small


def test_single_station_is_not_improved_by_themselves():
    case = get_case("single_station")
    out = monte_carlo(case, n=200, seed=1)
    assert out["fused"] is None            # 单站没有交会
    assert out["single"]["n"] >= 100
    assert out["level_assumption"]["mean_m"] > 5.0 * out["single"]["mean_m"]


def test_orthogonal_fusion_beats_single_station():
    case = get_case("two_station_orthogonal")
    out = monte_carlo(case, n=300, seed=2)
    assert out["fused"] is not None
    assert out["fused"]["mean_m"] < 0.85 * out["single"]["mean_m"]


def test_shallow_intersection_is_worse_than_single_station():
    """12° 开角：交会不但没帮忙，反而把偏置放大成几米——这是本模块的关键结论。"""
    case = get_case("two_station_shallow")
    out = monte_carlo(case, n=300, seed=3)
    assert out["fused"]["mean_m"] > 2.0 * out["single"]["mean_m"]


def test_case_geometry_metadata():
    single = get_case("single_station")
    orth = get_case("two_station_orthogonal")
    shallow = get_case("two_station_shallow")
    assert single.n_stations == 1
    assert math.isnan(single.intersection_deg)
    assert orth.intersection_deg == pytest.approx(90.0, abs=1e-6)
    assert shallow.intersection_deg == pytest.approx(12.0, abs=1e-6)


def test_case_validation():
    with pytest.raises(ValueError):
        GeoCase(name="bad", title="bad", stations=())
    with pytest.raises(ValueError):
        GeoCase(name="bad", title="bad", stations=(Station(0.0),), sigma_deg=-1.0)
    with pytest.raises(ValueError):
        GeoCase(name="bad", title="bad", stations=(Station(0.0),), samples=0)
    with pytest.raises(KeyError):
        get_case("no_such_case")


def test_station_position_uses_azimuth_counter_clockwise():
    p = Station(90.0, radius_m=10.0, height_m=5.0).position
    assert p == pytest.approx([0.0, 10.0, 5.0], abs=1e-9)


def test_from_station_directions_reports_per_station_and_fused():
    truth = (0.0, 0.0, 0.0)
    ps = [Station(0.0).position, Station(90.0).position]
    dirs = [los_to(truth, p) for p in ps]
    out = from_station_directions(ps, dirs, truth)
    assert out["fused_ok"]
    assert out["fused_m"] == pytest.approx(0.0, abs=1e-9)
    assert out["per_station_m"] == [0.0, 0.0]
    assert out["intersection_deg"] == pytest.approx(90.0, abs=1e-6)


def test_sweep_angle_returns_aligned_columns():
    angles = [5.0, 30.0, 90.0]
    out = sweep_angle(angles, sigma_deg=0.3, bias_deg=0.2, samples=60, n=60)
    assert out["angle_deg"] == angles
    assert len(out["fused_cep95_m"]) == 3
    # 开角越小越差：5° 一定比 90° 差
    assert out["fused_cep95_m"][0] > out["fused_cep95_m"][-1]


def test_monte_carlo_rejects_tiny_sample():
    case = get_case("single_station")
    with pytest.raises(ValueError):
        monte_carlo(case, n=5)


def test_available_cases_are_consistent():
    assert len(CASES) == 3
    names = {c.name for c in CASES}
    assert names == {"single_station", "two_station_orthogonal",
                     "two_station_shallow"}
    assert sweep_case(45.0).n_stations == 2
