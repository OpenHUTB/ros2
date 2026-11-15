"""Randomized world generator for the DWA stress benchmark.

Each world is a bounded 1.6 m half-width (3.2 m wide) corridor filled with ``count``
circular obstacles.  A world is accepted only if a ground-truth A* search with
the same inflation as the planner can connect start and goal: this guarantees
that a failure of the benchmark is a planner failure, not an impossible world.
"""

from __future__ import annotations

import math
import os
import random
import sys

try:
    from ros2_dwa_avoidance.dwa_core import DWAParams
    from ros2_dwa_avoidance.global_planner import OccupancyGrid, astar
    from ros2_dwa_avoidance.sim import Circle, Scenario, Segment
except ModuleNotFoundError as exc:
    if not (exc.name or "").startswith("ros2_dwa_avoidance"):
        raise
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                    "ros2_dwa_avoidance"))
    from ros2_dwa_avoidance.dwa_core import DWAParams
    from ros2_dwa_avoidance.global_planner import OccupancyGrid, astar
    from ros2_dwa_avoidance.sim import Circle, Scenario, Segment

START = (-0.5, 0.0, 0.0)
GOAL = (11.5, 0.0)
WALL_Y = 1.6


def _mark_circle(grid: OccupancyGrid, cx: float, cy: float, radius: float) -> None:
    r_cells = int(math.ceil(radius / grid.resolution))
    gx, gy = grid.world_to_grid(cx, cy)
    for dx in range(-r_cells, r_cells + 1):
        for dy in range(-r_cells, r_cells + 1):
            nx, ny = gx + dx, gy + dy
            if not grid.in_bounds(nx, ny):
                continue
            wx, wy = grid.grid_to_world(nx, ny)
            if math.hypot(wx - cx, wy - cy) <= radius:
                grid.occupied[ny, nx] = True


def _mark_segment(grid: OccupancyGrid, segment, radius: float) -> None:
    """Mark a thick line along a wall segment (sampled at half a cell)."""
    length = math.hypot(segment.x2 - segment.x1, segment.y2 - segment.y1)
    steps = max(2, int(math.ceil(length / (grid.resolution * 0.5))))
    for k in range(steps + 1):
        t = k / steps
        x = segment.x1 + (segment.x2 - segment.x1) * t
        y = segment.y1 + (segment.y2 - segment.y1) * t
        _mark_circle(grid, x, y, radius)


def is_feasible(scenario: Scenario, params: DWAParams) -> bool:
    grid = OccupancyGrid(-2.0, -WALL_Y - 1.0, 14.0, WALL_Y + 1.0, resolution=0.10)
    inflation = params.robot_radius + params.safety_margin
    for circle in scenario.circles:
        _mark_circle(grid, circle.x, circle.y, circle.r + inflation)
    # The corridor walls are segments, not circles: without them A* can leave
    # the corridor around the wall ends and "solve" worlds that are physically
    # enclosed.
    for segment in scenario.segments:
        _mark_segment(grid, segment, inflation)
    path = astar(grid, scenario.start[:2], scenario.goal)
    return bool(path)


def generate_world(seed: int, count: int, params: DWAParams,
                   attempts: int = 60) -> Scenario | None:
    """Return a feasible random world with ``count`` circles, or None."""
    for attempt in range(attempts):
        rng = random.Random(seed * 1000 + attempt)
        circles = []
        guard = 0
        while len(circles) < count and guard < 800:
            guard += 1
            cx = rng.uniform(0.8, 10.8)
            cy = rng.uniform(-(WALL_Y - 0.35), WALL_Y - 0.35)
            radius = rng.uniform(0.18, 0.30)
            if math.hypot(cx - START[0], cy - START[1]) < 1.2:
                continue
            if math.hypot(cx - GOAL[0], cy - GOAL[1]) < 1.2:
                continue
            if any(math.hypot(cx - c.x, cy - c.y) < radius + c.r + 0.55
                   for c in circles):
                continue
            circles.append(Circle(cx, cy, radius))
        if len(circles) < count:
            continue
        scenario = Scenario(
            name=f"random_{count}_{seed}",
            start=START,
            goal=GOAL,
            circles=circles,
            segments=[
                Segment(-2.0, WALL_Y, 14.0, WALL_Y),
                Segment(-2.0, -WALL_Y, 14.0, -WALL_Y),
            ],
            timeout=60.0,
            description=f"{count} random circles, seed {seed}",
        )
        if is_feasible(scenario, params):
            return scenario
    return None


