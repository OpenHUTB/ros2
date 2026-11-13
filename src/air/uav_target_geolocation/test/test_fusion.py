# -*- coding: utf-8 -*-
"""交会解与统计：无噪声必须精确、共线必须判奇异、权重必须有效。"""
import math

import numpy as np
import pytest

from uav_target_geolocation.core.fusion import (angular_spread_deg, cep, fuse,
                                                fused_from_directions,
                                                mean_direction, radial_errors,
                                                summarize)
from uav_target_geolocation.core.geometry import los_to, make_ray, station_position

TRUTH = np.array([0.0, 0.0, 0.0])


def test_orthogonal_rays_recover_truth_exactly():
    ps = [station_position(0.0, 60.0, 80.0), station_position(90.0, 60.0, 80.0)]
    dirs = [los_to(TRUTH, p) for p in ps]
    res = fused_from_directions(ps, dirs)
    assert res.ok
    assert res.error_to(TRUTH) == pytest.approx(0.0, abs=1e-9)


def test_three_stations_recover_truth_exactly():
    ps = [station_position(a, 60.0, 80.0) for a in (0.0, 120.0, 240.0)]
    dirs = [los_to(TRUTH, p) for p in ps]
    res = fused_from_directions(ps, dirs)
    assert res.ok
    assert res.error_to(TRUTH) == pytest.approx(0.0, abs=1e-9)
    assert res.residual_m == pytest.approx(0.0, abs=1e-9)


def test_parallel_rays_are_flagged_not_silently_solved():
    """两条近乎同向的视线：法方程病态，必须明确标出来。"""
    ps = [station_position(0.0, 60.0, 80.0), station_position(0.0, 60.0, 80.0)]
    dirs = [los_to(TRUTH, p) for p in ps]
    res = fused_from_directions(ps, dirs)
    assert not res.ok
    assert "共线" in res.reason or "奇异" in res.reason


def test_single_ray_is_not_reported_as_an_intersection():
    """一条视线只能定出一条线，定不出点——必须判奇异，不能给个假解。"""
    p = station_position(0.0, 60.0, 80.0)
    res = fused_from_directions([p], [los_to(TRUTH, p)])
    assert res.n_rays == 1
    assert not res.ok
    assert "共线" in res.reason or "奇异" in res.reason


def test_down_weighting_an_outlier_restores_the_truth():
    """两条正常视线 + 一条离群视线：给它极小的权重，解应回到真值。"""
    good = [make_ray(station_position(0.0, 60.0, 80.0), TRUTH),
            make_ray(station_position(90.0, 60.0, 80.0), TRUTH)]
    outlier = make_ray(station_position(45.0, 60.0, 80.0), (10.0, -10.0, 0.0))

    equal = fuse(good + [outlier], weights=[1.0, 1.0, 1.0])
    assert equal.ok
    assert float(np.linalg.norm(equal.point)) > 0.5      # 被离群值带偏

    down = fuse(good + [outlier], weights=[1.0, 1.0, 1e-6])
    assert float(np.linalg.norm(down.point)) < 0.05      # 回到真值


def test_fuse_rejects_bad_weights():
    p = station_position(0.0, 60.0, 80.0)
    ray = make_ray(p, TRUTH)
    with pytest.raises(ValueError):
        fuse([ray], weights=[0.0])
    with pytest.raises(ValueError):
        fuse([ray, ray], weights=[1.0])
    with pytest.raises(ValueError):
        fuse([])


def test_mean_direction_cancels_zero_mean_jitter():
    base = los_to(TRUTH, station_position(0.0, 60.0, 80.0))
    rng = np.random.default_rng(0)
    dirs = []
    for _ in range(400):
        a, b = rng.normal(0.0, 0.3, 2)
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(math.radians(a)), -math.sin(math.radians(a))],
                       [0, math.sin(math.radians(a)), math.cos(math.radians(a))]])
        Ry = np.array([[math.cos(math.radians(b)), 0, math.sin(math.radians(b))],
                       [0, 1, 0],
                       [-math.sin(math.radians(b)), 0, math.cos(math.radians(b))]])
        dirs.append(Rx @ Ry @ base)
    m = mean_direction(dirs)
    ang = math.degrees(math.acos(min(1.0, float(np.dot(m, base)))))
    assert ang < 0.1                          # 400 帧平均后偏不到 0.1°


def test_mean_direction_rejects_degenerate_input():
    with pytest.raises(ValueError):
        mean_direction([])
    with pytest.raises(ValueError):
        mean_direction([(0.0, 0.0, 0.0)])
    with pytest.raises(ValueError):
        mean_direction([(0.0, 0.0, -1.0), (0.0, 0.0, 1.0)])


def test_angular_spread_of_identical_directions_is_zero():
    d = los_to(TRUTH, station_position(0.0, 60.0, 80.0))
    assert angular_spread_deg([d, d, d]) == pytest.approx(0.0, abs=1e-12)


def test_radial_errors_and_summary():
    errs = radial_errors([(3.0, 4.0, 0.0), (0.0, 0.0, 0.0)], TRUTH)
    assert errs[0] == pytest.approx(5.0)
    assert errs[1] == pytest.approx(0.0)
    s = summarize([0.0, 1.0, 2.0, 3.0])
    assert s["n"] == 4
    assert s["mean_m"] == pytest.approx(1.5)
    assert s["max_m"] == pytest.approx(3.0)
    assert s["cep50_m"] == pytest.approx(float(np.percentile([0, 1, 2, 3], 50)))
    assert cep([1.0, 2.0], 0.95) == pytest.approx(float(np.percentile([1, 2], 95)))


def test_summary_rejects_empty_and_non_finite():
    with pytest.raises(ValueError):
        summarize([])
    with pytest.raises(ValueError):
        summarize([1.0, float("nan")])
