# -*- coding: utf-8 -*-
"""距离栅格与特征的单元测试。"""
import math

import numpy as np

from uav_dynamic_safety_guard.core import ranges as R


def test_empty_cloud_is_max_range():
    g = R.points_to_grid(np.zeros((0, 3)))
    assert g.shape == (R.N_EL, R.N_AZ)
    assert np.all(g == R.MAX_RANGE)


def test_point_lands_in_expected_cell():
    g = R.points_to_grid(np.array([[2.0, 0.0, 0.0]]))
    ai = int(round((0.0 + math.pi) / (2 * math.pi) * R.N_AZ)) % R.N_AZ
    assert g[1, ai] == 2.0
    assert int(np.sum(g < R.MAX_RANGE)) == 1


def test_elevation_goes_to_upper_and_lower_layers():
    up = R.points_to_grid(np.array([[2.0, 0.0, 1.5]]))
    down = R.points_to_grid(np.array([[2.0, 0.0, -1.5]]))
    assert up[2].min() < R.MAX_RANGE
    assert down[0].min() < R.MAX_RANGE


def test_out_of_range_points_are_ignored():
    g = R.points_to_grid(np.array([[R.MAX_RANGE + 1.0, 0.0, 0.0]]))
    assert np.all(g == R.MAX_RANGE)


def test_feature_dims_match_network_inputs():
    hist = np.full((R.HISTORY, R.N_EL, R.N_AZ), 2.0, np.float32)
    f1 = R.build_features(hist[0], np.zeros(3), np.array([1.0, 0.0, 0.0]), 4, 1.6)
    f3 = R.build_sequence_features(hist, np.zeros(3), np.array([1.0, 0.0, 0.0]),
                                   4, 1.6)
    assert f1.shape == (R.FEAT_DIM,)
    assert f3.shape == (R.FEAT_DIM_SEQ,)
    assert R.FEAT_DIM == 116 and R.FEAT_DIM_SEQ == 332


def test_mirror_swaps_left_and_right():
    g = np.full((R.N_EL, R.N_AZ), R.MAX_RANGE, np.float32)
    g[:, 5] = 1.0                     # 左侧某个方位
    m = R.mirror_grid(g)
    assert int(np.count_nonzero(m < R.MAX_RANGE)) == R.N_EL
    assert abs(float(m[0, R.MIRROR_IDX[5]]) - 1.0) < 1e-6
    assert np.allclose(R.mirror_grid(m), g)


def test_corrupt_grid_is_bounded():
    g = np.full((R.N_EL, R.N_AZ), 4.0, np.float32)
    c = R.corrupt_grid(g, np.random.default_rng(0), sigma=1.0, drop=0.3)
    assert c.min() >= 0.0 and c.max() <= R.MAX_RANGE
    assert np.any(c != g)


def test_direction_range_ignores_vertical_commands():
    g = np.full((R.N_EL, R.N_AZ), 3.0, np.float32)
    assert R.direction_range(g, np.array([0.0, 0.0, 1.5])) == (1.0, 1.0)
    center, near = R.direction_range(g, np.array([1.0, 0.0, 0.0]))
    assert center == 3.0 / R.MAX_RANGE
    assert near == 3.0 / R.MAX_RANGE


def test_lidar_lower_bound_shrinks_with_speed():
    g = np.full((R.N_EL, R.N_AZ), 4.0, np.float32)
    a = R.lidar_lower_bound(g, np.array([1.0, 0.0, 0.0]), 0.6)
    b = R.lidar_lower_bound(g, np.array([3.0, 0.0, 0.0]), 0.6)
    assert b < a


def test_grid_min_range():
    g = np.full((R.N_EL, R.N_AZ), 5.0, np.float32)
    g[2, 7] = 0.8
    assert abs(R.grid_min_range(g) - 0.8) < 1e-6
