"""Incremental scan mapping and route planning for the roundabout simulation."""

from __future__ import annotations

import math
import time

from .slam_nav import OccupancyGrid, PlanResult


class GridNavigator:
    """Update an occupancy grid from LaserScan-like values and plan to a goal."""

    def __init__(
        self,
        goal: tuple[float, float] = (8.0, 0.0),
        resolution: float = 0.25,
        width: int = 100,
        height: int = 80,
        origin: tuple[float, float] = (-10.0, -10.0),
    ) -> None:
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        self.goal = goal
        self.resolution = resolution
        self.width, self.height = width, height
        self.origin = origin
        self.grid = OccupancyGrid(width, height)
        self.update_count = 0
        self.rays_integrated = 0
        self.map_update_seconds = 0.0
        self.planning_seconds = 0.0
        self.planning_max_seconds = 0.0
        self.plan_count = 0
        self.found_count = 0
        self.first_scan_stamp: float | None = None
        self.last_scan_stamp: float | None = None
        self.last_result = PlanResult()
        self.last_path: tuple[tuple[int, int], ...] = ()
        self.collision_frames = 0
        self.route_completed = False
        self.route_episodes = 1

    def world_to_cell(self, point: tuple[float, float]) -> tuple[int, int]:
        return (
            int(math.floor((point[0] - self.origin[0]) / self.resolution)),
            int(math.floor((point[1] - self.origin[1]) / self.resolution)),
        )

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        return (
            self.origin[0] + (cell[0] + 0.5) * self.resolution,
            self.origin[1] + (cell[1] + 0.5) * self.resolution,
        )

    def update_scan(
        self,
        pose: tuple[float, float, float],
        ranges: list[float] | tuple[float, ...],
        angle_min: float,
        angle_increment: float,
        range_max: float,
        stamp: float,
    ) -> PlanResult:
        started = time.perf_counter()
        x, y, yaw = pose
        start = self.world_to_cell((x, y))
        for index, measured in enumerate(ranges):
            if math.isnan(measured) or measured < 0.0:
                continue
            distance = min(measured, range_max)
            angle = yaw + angle_min + index * angle_increment
            endpoint = self.world_to_cell((x + distance * math.cos(angle), y + distance * math.sin(angle)))
            hit = math.isfinite(measured) and measured < range_max - 1e-6
            self.grid.integrate_ray(start, endpoint, hit=hit)
            self.rays_integrated += 1

        self.map_update_seconds += time.perf_counter() - started
        self.update_count += 1
        if self.first_scan_stamp is None:
            self.first_scan_stamp = stamp
        self.last_scan_stamp = stamp

        planning_started = time.perf_counter()
        goal_cell = self.world_to_cell(self.goal)
        self.last_result = self.grid.plan(start, goal_cell)
        self.last_path = self.last_result.path
        duration = time.perf_counter() - planning_started
        self.planning_seconds += duration
        self.planning_max_seconds = max(self.planning_max_seconds, duration)
        self.plan_count += 1
        self.found_count += int(self.last_result.found)
        return self.last_result

    def record_collision(self, collision: bool) -> None:
        self.collision_frames += int(collision)

    def finish_episode(self, completed: bool) -> None:
        self.route_completed = bool(completed)

    def metrics(self) -> dict[str, float | int | bool | str]:
        elapsed = 0.0
        if self.first_scan_stamp is not None and self.last_scan_stamp is not None:
            elapsed = max(0.0, self.last_scan_stamp - self.first_scan_stamp)
        map_hz = (self.update_count - 1) / elapsed if elapsed > 0.0 else 0.0
        return {
            "map_resolution_m": self.resolution,
            "map_updates": self.update_count,
            "map_update_frequency_hz": round(map_hz, 3),
            "map_update_total_ms": round(self.map_update_seconds * 1000.0, 3),
            "map_rays_integrated": self.rays_integrated,
            "map_free_cells": sum(row.count(OccupancyGrid.FREE) for row in self.grid.cells),
            "map_occupied_cells": sum(row.count(OccupancyGrid.OCCUPIED) for row in self.grid.cells),
            "unknown_cells_traversable": False,
            "planning_updates": self.plan_count,
            "planning_successes": self.found_count,
            "planning_success_rate": round(self.found_count / self.plan_count, 3) if self.plan_count else 0.0,
            "planning_average_ms": round(self.planning_seconds * 1000.0 / self.plan_count, 3) if self.plan_count else 0.0,
            "planning_max_ms": round(self.planning_max_seconds * 1000.0, 3),
            "planning_failure": self.last_result.failure.value if self.last_result.failure else "",
            "route_episodes": self.route_episodes,
            "route_completed": self.route_completed,
            "route_completion_rate": float(self.route_completed),
            "route_completion_basis": "rule_policy_simulator",
            "planned_path_drives_vehicle": False,
            "collision_frames": self.collision_frames,
            "collision_detected": self.collision_frames > 0,
        }


def raycast_circles(
    origin: tuple[float, float],
    yaw: float,
    circles: list[tuple[float, float, float]],
    rays: int = 25,
    max_range: float = 12.0,
) -> list[float]:
    """Generate deterministic 180-degree range measurements for a simulated scan."""
    if rays < 2 or max_range <= 0.0:
        raise ValueError("rays must be >= 2 and max_range must be positive")
    values = []
    for index in range(rays):
        angle = yaw - math.pi / 2.0 + math.pi * index / (rays - 1)
        dx, dy = math.cos(angle), math.sin(angle)
        nearest = max_range
        for cx, cy, radius in circles:
            ox, oy = origin[0] - cx, origin[1] - cy
            b = 2.0 * (ox * dx + oy * dy)
            c = ox * ox + oy * oy - radius * radius
            discriminant = b * b - 4.0 * c
            if discriminant < 0.0:
                continue
            root = math.sqrt(discriminant)
            for distance in ((-b - root) / 2.0, (-b + root) / 2.0):
                if 0.0 <= distance < nearest:
                    nearest = distance
        values.append(nearest)
    return values
