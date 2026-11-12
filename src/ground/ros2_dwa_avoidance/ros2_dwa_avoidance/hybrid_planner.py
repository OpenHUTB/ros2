"""Two-layer planner: A* on a local occupancy grid + DWA trajectory tracking.

Pure DWA is a *local* method: when the goal lies directly behind an obstacle it
has no way to decide which side to pass and can oscillate in front of the
obstacle.  The A* layer supplies a route around known obstacles; the DWA layer
turns that route into smooth, kinematically feasible velocity commands and
still reacts to obstacles that appear after the route was computed.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:  # package import
    from .dwa_core import DWA, DWAParams, RobotState, PlanResult
    from .global_planner import (OccupancyGrid, astar, line_of_sight,
                                 point_at_distance, simplify_path)
except ImportError:  # direct execution
    from dwa_core import DWA, DWAParams, RobotState, PlanResult
    from global_planner import (OccupancyGrid, astar, line_of_sight,
                                point_at_distance, simplify_path)


class AStarDWAController:
    """A* global route + DWA local tracking."""

    def __init__(self, params: DWAParams,
                 bounds: Tuple[float, float, float, float],
                 resolution: float = 0.10,
                 replan_period: float = 1.00,
                 lookahead: float = 1.20,
                 goal_tolerance: float = 0.25) -> None:
        self.params = params
        self.planner = DWA(params)
        self.bounds = bounds
        self.resolution = resolution
        self.replan_period = replan_period
        self.lookahead = lookahead
        self.goal_tolerance = goal_tolerance
        self.grid: Optional[OccupancyGrid] = None
        self._grid = OccupancyGrid(*bounds, resolution=resolution)
        self.path: List[Tuple[float, float]] = []
        self.target: Tuple[float, float] = (0.0, 0.0)
        self._time = 0.0
        self._next_replan = -1.0
        self.last_result: Optional[PlanResult] = None
        self.replans = 0

    def reset(self) -> None:
        self._grid = OccupancyGrid(*self.bounds, resolution=self.resolution)
        self.path = []
        self._next_replan = -1.0
        self.replans = 0

    def shift_window(self, bounds: Tuple[float, float, float, float]) -> None:
        """Move the rolling window while keeping the already-seen obstacles.

        Rebuilding an empty grid on every shift loses the walls that are still
        relevant but just outside the latest scan; A* then plans straight
        through them and the robot oscillates at the same obstacle.
        """
        new_grid = OccupancyGrid(*bounds, resolution=self.resolution)
        for gy, gx in np.argwhere(self._grid.occupied):
            wx, wy = self._grid.grid_to_world(int(gx), int(gy))
            ngx, ngy = new_grid.world_to_grid(wx, wy)
            if new_grid.in_bounds(ngx, ngy):
                new_grid.occupied[ngy, ngx] = True
        self._grid = new_grid
        self.bounds = bounds
        self.path = []

    def _build_grid(self, state: RobotState, points: np.ndarray) -> OccupancyGrid:
        """Accumulate new scan points into the persistent world-frame grid.

        Keeping the grid between replans matters: walls that have left the
        current 6 m scan must not be forgotten, otherwise A* happily plans
        through them again and the robot oscillates at the same corner.
        """
        inflate = self.params.robot_radius + self.params.safety_margin
        self._grid.add_points(points, inflate)
        self._grid.clear_disc(state.x, state.y,
                              self.params.robot_radius + 2.0 * self.resolution)
        return self._grid

    def _clip_to_bounds(self, state: RobotState, goal: Tuple[float, float]) -> Tuple[float, float]:
        """Clip the robot->goal segment to the local grid (keeps A* solvable).

        A goal outside the rolling window must not make ``astar`` return an
        empty path: the robot would fall back to driving straight at it and
        stall on the first obstacle.
        """
        x_min, y_min, x_max, y_max = self.bounds
        margin = 0.40
        dx, dy = goal[0] - state.x, goal[1] - state.y
        t = 1.0
        if dx > 1e-9:
            t = min(t, (x_max - margin - state.x) / dx)
        elif dx < -1e-9:
            t = min(t, (x_min + margin - state.x) / dx)
        if dy > 1e-9:
            t = min(t, (y_max - margin - state.y) / dy)
        elif dy < -1e-9:
            t = min(t, (y_min + margin - state.y) / dy)
        t = max(0.0, min(1.0, t))
        return (state.x + t * dx, state.y + t * dy)

    def _plan_route(self, state: RobotState, goal: Tuple[float, float],
                    points: np.ndarray) -> None:
        self.grid = self._build_grid(state, points)
        target = goal
        gx, gy = self.grid.world_to_grid(*goal)
        if not self.grid.in_bounds(gx, gy):
            target = self._clip_to_bounds(state, goal)
        raw = astar(self.grid, (state.x, state.y), target)
        if raw:
            self.path = simplify_path(self.grid, raw)
        else:
            # Goal outside the local window or temporarily enclosed: aim directly
            # at it and let DWA deal with whatever is in the way.
            self.path = [(state.x, state.y), goal]
        self.replans += 1

    def _lookahead_target(self, state: RobotState, goal: Tuple[float, float]) -> Tuple[float, float]:
        if not self.path:
            return goal
        remaining = math.hypot(goal[0] - state.x, goal[1] - state.y)
        if remaining <= self.lookahead:
            return goal
        target = point_at_distance(self.path, (state.x, state.y), self.lookahead)
        # Never aim through a wall: shrink the lookahead until the target is
        # directly visible from the robot.
        step = self.lookahead
        while step > 0.35 and not line_of_sight(self._grid, (state.x, state.y), target):
            step -= 0.15
            target = point_at_distance(self.path, (state.x, state.y), step)
        if math.hypot(target[0] - state.x, target[1] - state.y) < self.goal_tolerance + 1e-3:
            return goal
        return target

    def __call__(self, state: RobotState, goal: Tuple[float, float],
                 points: np.ndarray) -> Tuple[float, float]:
        self._time += self.params.dt
        if self._time >= self._next_replan:
            self._plan_route(state, goal, points)
            self._next_replan = self._time + self.replan_period

        if math.hypot(goal[0] - state.x, goal[1] - state.y) <= self.goal_tolerance:
            self.last_result = PlanResult(0.0, 0.0, np.array([[state.x, state.y]]), 0.0,
                                          0.0, 0.0, 0.0, 0.0,
                                          float(self.planner._min_clearance(state, points)),
                                          source="goal")
            return 0.0, 0.0

        self.target = self._lookahead_target(state, goal)
        result = self.planner.plan(state, self.target, points)
        self.last_result = result
        return result.v, result.w


def path_length(path: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


