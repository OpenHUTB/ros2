# -*- coding: utf-8 -*-
"""视线几何：求交、掠射角、敏感度。"""
import math

import numpy as np
import pytest

from uav_target_geolocation.core.geometry import (Ray, circle_ellipse,
                                                  horizontal_intersection_angle_deg,
                                                  los_to, make_ray,
                                                  pairwise_min_angle_deg,
                                                  rotate_by_angle_err,
                                                  station_position, unit)


def test_vertical_ray_hits_directly_below():
    r = Ray(origin=(0.0, 0.0, 50.0), direction=(0.0, 0.0, -1.0))
    p = r.ground_hit()
    assert p == pytest.approx([0.0, 0.0, 0.0])
    assert r.ground_range() == pytest.approx(0.0)
    assert r.slant_range() == pytest.approx(50.0)
    assert r.grazing_angle_deg() == pytest.approx(0.0)


def test_slant_ray_hits_at_expected_range():
    # 45° 掠射：水平射程等于高度
    h = 40.0
    r = Ray(origin=(0.0, 0.0, h), direction=(1.0, 0.0, -1.0))
    assert r.ground_range() == pytest.approx(h)
    assert r.grazing_angle_deg() == pytest.approx(45.0)
    p = r.ground_hit()
    assert p[0] == pytest.approx(h)


def test_upward_ray_is_rejected():
    r = Ray(origin=(0.0, 0.0, 10.0), direction=(0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        r.ground_hit()


def test_horizontal_ray_is_rejected():
    r = Ray(origin=(0.0, 0.0, 10.0), direction=(1.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        r.ground_hit()


def test_ground_above_camera_is_rejected():
    r = Ray(origin=(0.0, 0.0, 10.0), direction=(0.1, 0.0, -1.0))
    with pytest.raises(ValueError):
        r.ground_hit(ground_z=20.0)


def test_sensitivity_matches_numeric_derivative():
    h = 80.0
    theta = 36.87
    t = math.radians(theta)
    origin = (0.0, 0.0, h)
    d = (math.tan(t), 0.0, -1.0)
    r = Ray(origin=origin, direction=d)
    analytic = r.sensitivity_m_per_deg()

    eps = 1e-4  # 度
    r_up = Ray(origin=origin,
               direction=rotate_by_angle_err(r.direction, eps, axis="y"))
    r_dn = Ray(origin=origin,
               direction=rotate_by_angle_err(r.direction, -eps, axis="y"))
    numeric = abs(r_up.ground_hit()[0] - r_dn.ground_hit()[0]) / (2 * eps)
    assert analytic == pytest.approx(numeric, rel=0.02)


def test_sensitivity_grows_with_grazing_angle():
    """越掠射，同样的角误差被放得越大——这是单站定位在高空失效的根因。"""
    steep = Ray(origin=(0, 0, 80.0), direction=(0.2, 0.0, -1.0))
    shallow = Ray(origin=(0, 0, 80.0), direction=(3.0, 0.0, -1.0))
    assert shallow.sensitivity_m_per_deg() > 4 * steep.sensitivity_m_per_deg()


def test_los_to_and_make_ray_are_consistent():
    origin = (10.0, -5.0, 30.0)
    target = (0.0, 0.0, 0.0)
    d = los_to(target, origin)
    assert float(np.linalg.norm(d)) == pytest.approx(1.0)
    assert make_ray(origin, target).ground_hit() == pytest.approx(
        [0.0, 0.0, 0.0], abs=1e-9)


def test_unit_rejects_zero_and_non_finite():
    with pytest.raises(ValueError):
        unit((0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        unit((float("nan"), 0.0, 1.0))


def test_ray_rejects_wrong_dimension():
    with pytest.raises(ValueError):
        Ray(origin=(0.0, 0.0), direction=(0.0, 0.0, -1.0))


def test_intersection_angle_of_orthogonal_rays():
    a = make_ray((60.0, 0.0, 80.0), (0.0, 0.0, 0.0))
    b = make_ray((0.0, 60.0, 80.0), (0.0, 0.0, 0.0))
    assert horizontal_intersection_angle_deg([a, b]) == pytest.approx(90.0, abs=1e-6)


def test_intersection_angle_of_nearly_parallel_rays():
    a = make_ray((60.0, 0.0, 80.0), (0.0, 0.0, 0.0))
    b = make_ray(station_position(12.0, 60.0, 80.0), (0.0, 0.0, 0.0))
    assert horizontal_intersection_angle_deg([a, b]) == pytest.approx(12.0, abs=0.2)


def test_pairwise_min_angle_wraps_around_180():
    assert pairwise_min_angle_deg([0.0, 170.0]) == pytest.approx(10.0)
    assert pairwise_min_angle_deg([0.0, 90.0, 180.0]) == pytest.approx(0.0)
    assert math.isnan(pairwise_min_angle_deg([10.0]))


def test_circle_ellipse_of_isotropic_cov():
    a, b, ang = circle_ellipse(np.eye(2) * 4.0, n_sigma=2.0)
    assert a == pytest.approx(4.0) and b == pytest.approx(4.0)


def test_circle_ellipse_of_anisotropic_cov():
    cov = np.array([[9.0, 0.0], [0.0, 1.0]])
    a, b, ang = circle_ellipse(cov, n_sigma=1.0)
    assert a == pytest.approx(3.0) and b == pytest.approx(1.0)
    assert abs(ang) < 1e-9
