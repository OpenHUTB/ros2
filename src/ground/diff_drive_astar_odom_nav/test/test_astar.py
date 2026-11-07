#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A* 核心算法测试（不依赖 ROS / Gazebo，可直接 pytest）。"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from diff_drive_astar_odom_nav.astar import (  # noqa: E402
    find_path, inflate_grid, is_connected, simplify_collinear)


def test_empty_grid_straight_line():
    grid = np.zeros((20, 20), dtype=bool)
    path, info = find_path(grid, (0, 0), (0, 19))
    assert info["found"]
    assert path[0] == (0, 0) and path[-1] == (0, 19)
    assert len(path) == 20


def test_diagonal_path_is_used():
    grid = np.zeros((10, 10), dtype=bool)
    path, info = find_path(grid, (0, 0), (9, 9))
    assert info["found"]
    # 每步都沿对角线时，峰值代价应接近 9*sqrt(2)
    assert info["cost"] < 13.0


def test_obstacle_is_avoided():
    grid = np.zeros((20, 20), dtype=bool)
    grid[5:15, 10] = True
    path, info = find_path(grid, (10, 0), (10, 19))
    assert info["found"]
    assert all(not grid[r, c] for r, c in path)


def test_unreachable_goal_is_reported():
    grid = np.zeros((20, 20), dtype=bool)
    grid[:, 10] = True
    path, info = find_path(grid, (10, 0), (10, 19))
    assert not path
    assert "not connected" in info["reason"]
    assert not is_connected(grid, (10, 0), (10, 19))


def test_inflation_closes_narrow_gap():
    grid = np.zeros((20, 20), dtype=bool)
    grid[:, 9] = True                # 一道竖墙
    grid[9:12, 9] = False            # 留出 3 格宽的门
    inflated = inflate_grid(grid, 2)
    path, info = find_path(inflated, (10, 0), (10, 19))
    assert not path and "not connected" in info["reason"]

    inflated = inflate_grid(grid, 1)
    path, info = find_path(inflated, (10, 0), (10, 19))
    assert info["found"]


def test_strict_corner_cutting():
    grid = np.zeros((30, 30), dtype=bool)
    # 挡住对角方向两侧中的一侧，严格穿角规则必须拒绝该对角步
    grid[10, 11] = True
    grid[11, 10] = True
    path, info = find_path(grid, (10, 10), (11, 11))
    assert info["found"]
    assert path == [(10, 10), (10, 11), (11, 11)] or path == [(10, 10), (11, 10), (11, 11)] \
        or path[1] != (11, 11)


def test_start_in_obstacle_is_repaired():
    grid = np.zeros((20, 20), dtype=bool)
    grid[10, 10] = True
    path, info = find_path(grid, (10, 10), (10, 19))
    assert info["found"]
    assert path[0] != (10, 10)


def test_simplify_collinear_keeps_endpoints():
    path = [(0, 0), (0, 1), (0, 2), (1, 3)]
    simplified = simplify_collinear(path)
    assert simplified[0] == path[0] and simplified[-1] == path[-1]
    assert len(simplified) < len(path)
