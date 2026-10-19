"""Deterministic pose-graph-lite SLAM loop over the simulated range sensor."""

from __future__ import annotations

from dataclasses import dataclass
import math

from navigation import Cell, OccupancyGrid
from neural_navigation import NeuralAction, NeuralPlanner


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RangeHit:
    distance: float
    bearing: float


class IncrementalSlam:
    """Integrate odometry and range hits into one pose/map estimate."""

    def __init__(self, grid: OccupancyGrid, cell_size: float = 0.5) -> None:
        if cell_size <= 0.0:
            raise ValueError("cell_size must be positive")
        self.grid = grid
        self.cell_size = cell_size
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.trajectory: list[Pose2D] = [self.pose]

    def integrate_odometry(self, dx: float, dy: float, dyaw: float) -> Pose2D:
        """Apply body-frame odometry and append the estimated pose."""
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        self.pose = Pose2D(
            self.pose.x + c * dx - s * dy,
            self.pose.y + s * dx + c * dy,
            self.pose.yaw + dyaw,
        )
        self.trajectory.append(self.pose)
        return self.pose

    def update_scan(self, hits: list[RangeHit]) -> None:
        """Ray-cast a local scan using the current pose and update occupancy."""
        origin = self._cell(self.pose.x, self.pose.y)
        cells: list[Cell] = []
        for hit in hits:
            angle = self.pose.yaw + hit.bearing
            x = self.pose.x + hit.distance * math.cos(angle)
            y = self.pose.y + hit.distance * math.sin(angle)
            cells.append(self._cell(x, y))
        self.grid.mark_sensor_ray(origin, cells)

    def _cell(self, x: float, y: float) -> Cell:
        return (int(round(x / self.cell_size)) + self.grid.width // 2,
                int(round(y / self.cell_size)) + self.grid.height // 2)

class SynchronizedSlamNavigation:
    """Per-step odometry, scan integration and neural planning loop."""
    def __init__(self, grid: OccupancyGrid, goal: Cell, cell_size: float = 0.5) -> None:
        self.slam = IncrementalSlam(grid, cell_size)
        self.goal = goal
        self.planner = NeuralPlanner()
    def step(self, dx: float, dy: float, dyaw: float, hits: list[RangeHit]) -> NeuralAction:
        self.slam.integrate_odometry(dx, dy, dyaw)
        self.slam.update_scan(hits)
        current = self.slam._cell(self.slam.pose.x, self.slam.pose.y)
        return self.planner.predict(self.slam.grid, current, self.goal)
