"""平面拟合与坡度计算的单元测试，不需要 ROS 与 PyBullet。"""

import numpy as np
import pytest

from ground_slope_estimation.plane_fit import (
    fit_plane_least_squares,
    fit_plane_ransac,
    inlier_ratio,
    plane_height,
    slope_angle,
)


def tilted_plane(angle_deg, n=20, noise=0.0, seed=0):
    """生成绕 y 轴倾斜 angle_deg 的平面点云，上表面中心过原点。"""
    axis = np.linspace(-0.5, 0.5, n)
    xx, yy = np.meshgrid(axis, axis)
    theta = np.deg2rad(angle_deg)
    # 平面 z = x·tanθ，绕 y 轴正向旋转后法向量为 (sinθ, 0, cosθ)
    zz = xx * np.tan(theta)
    points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)
    if noise > 0.0:
        rng = np.random.default_rng(seed)
        points[:, 2] += rng.normal(0.0, noise, points.shape[0])
    return points


@pytest.mark.parametrize('angle', [0.0, 5.0, 10.0, 20.0])
def test_least_squares_recovers_slope(angle):
    points = tilted_plane(angle)
    normal, _ = fit_plane_least_squares(points)
    assert np.rad2deg(slope_angle(normal)) == pytest.approx(angle, abs=1e-9)


def test_normal_is_unit_and_points_up():
    normal, _ = fit_plane_least_squares(tilted_plane(15.0))
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert normal[2] > 0.0


def test_plane_height_matches_generator():
    angle = 12.0
    points = tilted_plane(angle)
    normal, d = fit_plane_least_squares(points)
    for x, y in ((0.0, 0.0), (0.3, -0.2), (-0.4, 0.25)):
        assert plane_height(normal, d, x, y) == pytest.approx(
            x * np.tan(np.deg2rad(angle)), abs=1e-9)


def test_noise_keeps_error_small():
    points = tilted_plane(10.0, n=40, noise=0.002, seed=1)
    normal, _ = fit_plane_least_squares(points)
    error = abs(np.rad2deg(slope_angle(normal)) - 10.0)
    assert error < 0.1


def test_ransac_rejects_outliers():
    clean = tilted_plane(8.0, n=30)
    rng = np.random.default_rng(3)
    outliers = rng.uniform(-0.5, 0.5, size=(40, 3))
    outliers[:, 2] += 0.8
    points = np.vstack([clean, outliers])
    normal, _, ratio = fit_plane_ransac(points, threshold=0.02, seed=0)
    assert abs(np.rad2deg(slope_angle(normal)) - 8.0) < 0.2
    assert 0.4 < ratio < 1.0


def test_inlier_ratio_extremes():
    points = tilted_plane(0.0, n=10)
    normal, d = fit_plane_least_squares(points)
    assert inlier_ratio(points, normal, d, 0.01) == pytest.approx(1.0)
    shifted = points + np.array([0.0, 0.0, 5.0])
    assert inlier_ratio(shifted, normal, d, 0.01) == pytest.approx(0.0)


def test_rejects_too_few_points():
    with pytest.raises(ValueError):
        fit_plane_least_squares(np.zeros((2, 3)))


def test_rejects_non_finite_points():
    points = tilted_plane(0.0, n=4)
    points[0, 2] = np.nan
    with pytest.raises(ValueError):
        fit_plane_least_squares(points)


def test_rejects_bad_shape():
    with pytest.raises(ValueError):
        fit_plane_least_squares(np.zeros((10, 2)))


def test_slope_angle_rejects_zero_vector():
    with pytest.raises(ValueError):
        slope_angle(np.zeros(3))


def test_plane_height_rejects_horizontal_normal():
    with pytest.raises(ValueError):
        plane_height(np.array([1.0, 0.0, 0.0]), 0.0, 0.0, 0.0)
