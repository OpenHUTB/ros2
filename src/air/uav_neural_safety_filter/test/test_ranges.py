import numpy as np

from uav_neural_safety_filter.core.ranges import (MAX_RANGE, N_AZ, N_EL,
                                                  corrupt_grid, direction_range,
                                                  lidar_lower_bound,
                                                  points_to_grid)


def test_grid_shape_and_front():
    g = points_to_grid(np.array([[1.0, 0.0, 0.0]]), max_range=8.0)
    assert g.shape == (N_EL, N_AZ)
    assert abs(g[1, N_AZ // 2] - 1.0) < 1e-6


def test_grid_keeps_min():
    pts = np.array([[2.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    g = points_to_grid(pts, max_range=8.0)
    assert abs(g[1, N_AZ // 2] - 1.0) < 1e-6


def test_direction_range_normalized():
    g = np.full((N_EL, N_AZ), MAX_RANGE, np.float32)
    c, nb = direction_range(g, [1.0, 0.0, 0.0])
    assert 0.0 < c <= 1.0 and 0.0 < nb <= 1.0


def test_vertical_has_no_direction():
    g = np.full((N_EL, N_AZ), 0.5, np.float32)
    assert direction_range(g, [0.0, 0.0, 1.0]) == (1.0, 1.0)


def test_lower_bound_le_range():
    g = np.full((N_EL, N_AZ), MAX_RANGE, np.float32)
    assert lidar_lower_bound(g, [2.0, 0.0, 0.0]) <= MAX_RANGE


def test_corrupt_drops_points():
    g = np.full((N_EL, N_AZ), 3.0, np.float32)
    rng = np.random.default_rng(0)
    c = corrupt_grid(g, rng, sigma=0.0, drop=0.5)
    assert np.any(c == MAX_RANGE)
