#!/usr/bin/env python3
"""任务③测试：占据栅格建图、Neural A* 与导航闭环。"""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

import numpy as np  # noqa: E402

from mujoco_auto_parking.lidar_sensor import Lidar2D  # noqa: E402
from mujoco_auto_parking.navigation import run_episode  # noqa: E402
from mujoco_auto_parking.neural_path_planner import (  # noqa: E402
    BaselineAStarPlanner,
    NeuralAStarPlanner,
)
from mujoco_auto_parking.occupancy_map import OccupancyGridMap  # noqa: E402
from mujoco_auto_parking.simulator import MujocoSimulation  # noqa: E402

MODEL_PATH = PACKAGE_ROOT / 'models' / 'parking_lot.xml'
PLANNER_WEIGHTS = PACKAGE_ROOT / 'models' / 'planner.npz'
CONTROLLER_WEIGHTS = PACKAGE_ROOT / 'models' / 'trajectory.npz'


def check_mapping() -> None:
    simulation = MujocoSimulation(MODEL_PATH)
    simulation.set_pose(-6.0, 0.0, 0.0)
    lidar = Lidar2D(simulation.model, simulation.data, num_beams=120)
    scan = lidar.scan()
    grid_map = OccupancyGridMap()
    grid_map.update(
        -6.0,
        0.0,
        0.0,
        scan['ranges'],
        lidar.angle_min,
        lidar.angle_increment,
        lidar.min_range,
        lidar.max_range,
    )
    stats = grid_map.statistics()
    assert stats['known_cells'] > 100, stats
    assert stats['occupied_cells'] > 0, stats
    ros_grid = grid_map.to_ros_grid()
    assert ros_grid.shape == (200, 200)
    print(
        f'[PASS] 建图：已观测 {stats["known_cells"]} 格，'
        f'占据 {stats["occupied_cells"]} 格，覆盖率 {stats["coverage_pct"]:.1f}%'
    )


def make_grid():
    grid = np.zeros((80, 80), dtype=np.int8)
    grid[20:60, 30:35] = 100
    grid[10:15, 10:60] = 100
    return grid


def check_planner() -> None:
    assert PLANNER_WEIGHTS.is_file(), PLANNER_WEIGHTS
    grid = make_grid()
    origin = (0.0, 0.0)
    start = (2.0, 2.0)
    goal = (7.0, 7.0)
    baseline = BaselineAStarPlanner(resolution=0.1, clearance_m=0.3)
    neural = NeuralAStarPlanner(
        resolution=0.1, clearance_m=0.3, weights_path=PLANNER_WEIGHTS
    )
    baseline_path = baseline.plan(grid, origin, start, goal)
    neural_path = neural.plan(grid, origin, start, goal)
    assert len(baseline_path) >= 2 and len(neural_path) >= 2
    assert neural.expanded_nodes <= baseline.expanded_nodes * 1.2, (
        baseline.expanded_nodes, neural.expanded_nodes
    )
    print(
        f'[PASS] Neural A*：展开节点 {neural.expanded_nodes} '
        f'(A* {baseline.expanded_nodes})，'
        f'规划 {neural.planning_time_ms:.1f} ms，路径 {len(neural_path)} 点'
    )


def check_closed_loop() -> None:
    assert CONTROLLER_WEIGHTS.is_file(), CONTROLLER_WEIGHTS
    simulation = MujocoSimulation(MODEL_PATH)
    lidar = Lidar2D(simulation.model, simulation.data, num_beams=120)
    for goal in ((0.25, 1.2), (0.25, -1.2)):
        metrics = run_episode(
            simulation,
            lidar,
            goal,
            PLANNER_WEIGHTS,
            CONTROLLER_WEIGHTS,
            max_time=60.0,
        )
        assert metrics['reached'] == 1.0, (goal, metrics)
        assert metrics['final_distance'] < 0.4, (goal, metrics)
        assert metrics['map_coverage_pct'] > 20.0, (goal, metrics)
        print(
            f'[PASS] 导航闭环 {goal}: 用时 {metrics["time"]:.1f} s，'
            f'终点误差 {metrics["final_distance"]:.2f} m，'
            f'建图覆盖 {metrics["map_coverage_pct"]:.1f}%，'
            f'规划节点 {metrics["expanded_nodes"]:.0f}，'
            f'最小离障 {metrics["min_clearance_m"]:.2f} m'
        )


def main() -> None:
    check_mapping()
    check_planner()
    check_closed_loop()
    print('\n任务③建图与导航测试全部通过。')


if __name__ == '__main__':
    main()
