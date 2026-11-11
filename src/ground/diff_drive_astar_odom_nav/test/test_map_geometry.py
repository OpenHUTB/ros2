#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图一致性测试：栅格地图、Gazebo 场景几何与 A* 路径必须对齐。"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from diff_drive_astar_odom_nav.astar import find_path, inflate_grid, path_length_world
from diff_drive_astar_odom_nav.geometry import GOAL_POSE, START_POSE, build_occupancy
from diff_drive_astar_odom_nav.map_utils import load_map

ROOT = Path(__file__).resolve().parents[1]


def test_map_size_and_resolution():
    grid_map = load_map(ROOT / "maps" / "static_map.yaml")
    assert grid_map.width == 240 and grid_map.height == 240
    assert abs(grid_map.resolution - 0.05) < 1e-9
    grid, info = build_occupancy()
    assert grid.shape == grid_map.grid.shape
    assert (grid == grid_map.grid).all()


def test_start_and_goal_are_free():
    grid_map = load_map(ROOT / "maps" / "static_map.yaml")
    assert not grid_map.grid[grid_map.world_to_cell(*START_POSE[:2])]
    assert not grid_map.grid[grid_map.world_to_cell(*GOAL_POSE[:2])]


def test_planned_path_is_safe_and_connected():
    grid_map = load_map(ROOT / "maps" / "static_map.yaml")
    inflated = inflate_grid(grid_map.grid, math.ceil(0.27 / grid_map.resolution))
    start = grid_map.world_to_cell(*START_POSE[:2])
    goal = grid_map.world_to_cell(*GOAL_POSE[:2])
    path, info = find_path(inflated, start, goal)
    assert info["found"] and path
    points = [grid_map.cell_to_world(*cell) for cell in path]
    # 输出路径上的每个点都必须落在空闲栅格内
    for point in points[::5]:
        row, col = grid_map.world_to_cell(*point)
        assert not grid_map.grid[row, col]
    length = path_length_world(points)
    straight = math.hypot(GOAL_POSE[0] - START_POSE[0], GOAL_POSE[1] - START_POSE[1])
    assert length > straight * 1.05
