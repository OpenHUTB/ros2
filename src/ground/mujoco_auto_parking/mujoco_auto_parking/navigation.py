"""Mapping + Neural A* planning + neural path-following navigation core."""

import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .kinematics import DriveGeometry
from .neural_path_planner import BaselineAStarPlanner, NeuralAStarPlanner
from .occupancy_map import OccupancyGridMap
from .trajectory_control import (
    ReferenceTrajectory,
    TrajectoryController,
    wrap_angle,
)


def densify_path(path: List[Tuple[float, float]], step: float = 0.2):
    """Insert points so consecutive waypoints are at most `step` apart."""
    if len(path) < 2:
        return [(float(path[0][0]), float(path[0][1]))] if path else []
    dense = [(float(path[0][0]), float(path[0][1]))]
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        distance = math.hypot(x1 - x0, y1 - y0)
        count = max(1, int(math.ceil(distance / step)))
        for index in range(1, count + 1):
            ratio = index / count
            dense.append((
                float(x0) + ratio * (x1 - x0),
                float(y0) + ratio * (y1 - y0),
            ))
    return dense


def path_to_trajectory(path: List[Tuple[float, float]]) -> Optional[ReferenceTrajectory]:
    """Convert a world-frame polyline into a pose trajectory (x, y, yaw)."""
    dense = densify_path(path)
    if len(dense) < 2:
        return None
    points = []
    for index, (x, y) in enumerate(dense):
        if index < len(dense) - 1:
            nx, ny = dense[index + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            px, py = dense[index - 1]
            yaw = math.atan2(y - py, x - px)
        points.append([x, y, yaw])
    return ReferenceTrajectory(np.asarray(points, dtype=np.float64))


class NavigationCore:
    """Online mapping, Neural A* replanning and neural path following."""

    def __init__(
        self,
        planner_weights: Path | str | None = None,
        controller_weights: Path | str | None = None,
        resolution: float = 0.1,
        clearance_m: float = 0.4,
    ) -> None:
        self.resolution = float(resolution)
        self.map = OccupancyGridMap(
            width=int(20.0 / self.resolution),
            height=int(20.0 / self.resolution),
            resolution=self.resolution,
            origin=(-10.0, -10.0),
        )
        self.baseline = BaselineAStarPlanner(
            resolution=self.resolution, clearance_m=clearance_m
        )
        self.neural = NeuralAStarPlanner(
            resolution=self.resolution,
            clearance_m=clearance_m,
            weights_path=planner_weights,
        )
        self.controller = TrajectoryController(controller_weights)
        self.geometry = DriveGeometry(0.08, 0.48, 8.0)
        self.path: List[Tuple[float, float]] = []
        self.trajectory: Optional[ReferenceTrajectory] = None
        self.last_plan_time = -1e9
        self.last_planning_stats: Dict[str, float] = {}

    def update_map(self, state, scan) -> None:
        self.map.update(
            state.x,
            state.y,
            state.yaw,
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )

    def plan(self, state, goal_world: Tuple[float, float]) -> None:
        grid = self.map.to_occupancy_grid(unknown_as_free=True)
        start = (float(state.x), float(state.y))
        self.path = self.neural.plan(
            grid, self.map.origin, start, (float(goal_world[0]), float(goal_world[1]))
        )
        self.trajectory = path_to_trajectory(self.path)
        self.last_planning_stats = {
            'expanded_nodes': int(self.neural.expanded_nodes),
            'planning_time_ms': float(self.neural.planning_time_ms),
            'path_points': int(len(self.path)),
            'path_length_m': float(
                sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(self.path[:-1], self.path[1:])
                )
            ),
        }
        self.last_plan_time = time.monotonic()

    def control(self, state) -> Tuple[float, float]:
        if self.trajectory is None:
            return 0.0, 0.0
        _, _, reference = self.trajectory.lookahead(state.x, state.y)
        return self.controller.predict(
            (state.x, state.y, state.yaw, state.linear_velocity),
            reference,
            0.45,
        )


def run_episode(
    simulation,
    lidar,
    goal_world: Tuple[float, float],
    planner_weights: Path | str | None,
    controller_weights: Path | str | None,
    start_pose: Tuple[float, float, float] = (-6.0, 0.0, 0.0),
    max_time: float = 60.0,
    replan_period: float = 1.0,
) -> Dict[str, float]:
    """Run a standalone mapping/planning/control episode for testing."""
    core = NavigationCore(planner_weights, controller_weights)
    simulation.reset()
    simulation.set_pose(*start_pose)
    control_period = 0.02
    steps_per_control = max(1, int(round(control_period / simulation.timestep)))
    time_elapsed = 0.0
    reached = False
    min_clearance = 99.0
    path_length = 0.0

    while time_elapsed < max_time:
        state = simulation.planar_state()
        scan_dict = lidar.scan()
        ranges = scan_dict['ranges']

        class _Scan:
            pass

        scan = _Scan()
        scan.ranges = ranges
        scan.angle_min = lidar.angle_min
        scan.angle_increment = lidar.angle_increment
        scan.range_min = lidar.min_range
        scan.range_max = lidar.max_range
        core.update_map(state, scan)
        min_clearance = min(min_clearance, float(np.min(ranges)))

        if time_elapsed - core.last_plan_time >= replan_period:
            core.plan(state, goal_world)
            path_length = core.last_planning_stats.get('path_length_m', 0.0)

        speed, yaw_rate = core.control(state)
        if float(np.min(ranges)) < 0.35:
            speed = 0.0
        left, right = core.geometry.wheel_speeds(speed, yaw_rate)
        for _ in range(steps_per_control):
            simulation.apply_wheel_speeds(left, right)
            simulation.step()

        distance = math.hypot(state.x - goal_world[0], state.y - goal_world[1])
        if distance < 0.35:
            reached = True
            break
        time_elapsed += control_period

    stats = core.map.statistics()
    return {
        'reached': float(1.0 if reached else 0.0),
        'time': float(time_elapsed),
        'final_distance': float(distance),
        'map_coverage_pct': float(stats['coverage_pct']),
        'occupied_cells': float(stats['occupied_cells']),
        'expanded_nodes': float(core.last_planning_stats.get('expanded_nodes', 0)),
        'planning_time_ms': float(core.last_planning_stats.get('planning_time_ms', 0.0)),
        'path_length_m': float(path_length),
        'min_clearance_m': float(min_clearance),
    }
