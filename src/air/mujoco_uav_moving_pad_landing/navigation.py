"""Lightweight 2D lidar mapping and replanning demonstration."""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw

Cell = tuple[int, int]


@dataclass(frozen=True)
class GridWorld:
    """Small simulated flight area with fixed rectangular obstacles."""

    width: int = 24
    height: int = 16
    obstacles: tuple[tuple[int, int, int, int], ...] = (
        (7, 1, 8, 9),
        (12, 6, 16, 7),
        (19, 7, 20, 14),
    )

    @property
    def start(self) -> Cell:
        return (2, 2)

    @property
    def goal(self) -> Cell:
        return (21, 13)

    def occupied(self, cell: Cell) -> bool:
        x, y = cell
        if x <= 0 or y <= 0 or x >= self.width - 1 or y >= self.height - 1:
            return True
        return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in self.obstacles)


class OccupancyGrid:
    """Unknown cells are -1, free cells are 0, and occupied cells are 100."""

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.data = [[-1 for _ in range(width)] for _ in range(height)]

    def contains(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def update_scan(
        self,
        world: GridWorld,
        pose: Cell,
        angle_count: int = 360,
        max_range: float = 32.0,
    ) -> None:
        """Cast simulated lidar rays and mark traversed cells in the local map."""
        for ray in range(angle_count):
            angle = 2.0 * math.pi * ray / angle_count
            endpoint, hit = cast_ray(world, pose, angle, max_range)
            cells = bresenham(pose, endpoint)
            free_cells = cells[:-1] if hit else cells
            for cell in free_cells:
                if self.contains(cell) and self.data[cell[1]][cell[0]] != 100:
                    self.data[cell[1]][cell[0]] = 0
            if hit and self.contains(endpoint):
                self.data[endpoint[1]][endpoint[0]] = 100

    def coverage(self) -> float:
        known = sum(value >= 0 for row in self.data for value in row)
        return known / (self.width * self.height)


def cast_ray(world: GridWorld, pose: Cell, angle: float, max_range: float) -> tuple[Cell, bool]:
    """Return the first occupied cell intersected by one simulated lidar ray."""
    distance = 0.25
    while distance <= max_range:
        x = int(round(pose[0] + math.cos(angle) * distance))
        y = int(round(pose[1] + math.sin(angle) * distance))
        cell = (x, y)
        if world.occupied(cell):
            return cell, True
        distance += 0.25
    return (
        int(round(pose[0] + math.cos(angle) * max_range)),
        int(round(pose[1] + math.sin(angle) * max_range)),
    ), False


def bresenham(start: Cell, end: Cell) -> list[Cell]:
    """List the integer grid cells crossed by a ray between two endpoints."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    cells: list[Cell] = []
    while True:
        cells.append((x0, y0))
        if (x0, y0) == (x1, y1):
            return cells
        doubled_error = 2 * error
        if doubled_error > -dy:
            error -= dy
            x0 += sx
        if doubled_error < dx:
            error += dx
            y0 += sy


def astar(grid: OccupancyGrid, start: Cell, goal: Cell) -> list[Cell]:
    """Prefer observed free cells and assign a higher traversal cost to unknowns."""
    frontier: list[tuple[int, int, Cell]] = [(0, 0, start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    costs: dict[Cell, int] = {start: 0}
    sequence = 0
    while frontier:
        _priority, _order, current = heapq.heappop(frontier)
        if current == goal:
            path = [current]
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            return list(reversed(path))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if not grid.contains(neighbor):
                continue
            cell_value = grid.data[neighbor[1]][neighbor[0]]
            if cell_value == 100:
                continue
            next_cost = costs[current] + (4 if cell_value < 0 else 1)
            if neighbor in costs and next_cost >= costs[neighbor]:
                continue
            costs[neighbor] = next_cost
            came_from[neighbor] = current
            heuristic = abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1])
            sequence += 1
            heapq.heappush(frontier, (next_cost + heuristic, sequence, neighbor))
    return []


def render_map(
    grid: OccupancyGrid,
    path: list[Cell],
    visited: list[Cell],
    start: Cell,
    goal: Cell,
    cell_size: int = 18,
) -> Image.Image:
    """Render the current occupancy map and navigation progress."""
    image = Image.new("RGB", (grid.width * cell_size, grid.height * cell_size), "white")
    draw = ImageDraw.Draw(image)
    colors = {-1: (185, 190, 194), 0: (248, 249, 247), 100: (48, 57, 59)}
    for y, row in enumerate(grid.data):
        for x, value in enumerate(row):
            left, top = x * cell_size, (grid.height - y - 1) * cell_size
            draw.rectangle((left, top, left + cell_size - 1, top + cell_size - 1), fill=colors[value])
    for cell in path:
        x, y = cell
        draw.rectangle((x * cell_size + 5, (grid.height - y - 1) * cell_size + 5,
                        (x + 1) * cell_size - 5, (grid.height - y) * cell_size - 5), fill=(42, 150, 166))
    for cell in visited:
        x, y = cell
        draw.ellipse((x * cell_size + 5, (grid.height - y - 1) * cell_size + 5,
                      (x + 1) * cell_size - 5, (grid.height - y) * cell_size - 5), fill=(60, 155, 97))
    for cell, color in ((start, (42, 100, 210)), (goal, (211, 73, 63))):
        x, y = cell
        draw.ellipse((x * cell_size + 2, (grid.height - y - 1) * cell_size + 2,
                      (x + 1) * cell_size - 3, (grid.height - y) * cell_size - 3), fill=color)
    return image


def run_slam_navigation(output_dir: Path) -> dict[str, int | float | bool]:
    """Map, localize by simulated odometry, and replan while following a route."""
    output_dir.mkdir(parents=True, exist_ok=True)
    world = GridWorld()
    grid = OccupancyGrid(world.width, world.height)
    estimated_pose = world.start
    visited = [estimated_pose]
    frames: list[Image.Image] = []
    grid.update_scan(world, estimated_pose)
    route = astar(grid, estimated_pose, world.goal)
    if not route:
        raise RuntimeError("initial lidar scan did not reveal a route to the goal")

    for _ in range(world.width * world.height):
        if estimated_pose == world.goal:
            break
        route = astar(grid, estimated_pose, world.goal)
        if len(route) < 2:
            raise RuntimeError("no safe route remains in the observed occupancy map")
        next_pose = route[1]
        estimated_pose = next_pose
        visited.append(estimated_pose)
        grid.update_scan(world, estimated_pose)
        frames.append(render_map(grid, route, visited, world.start, world.goal))

    route = astar(grid, world.start, world.goal)
    final_map = render_map(grid, route, visited, world.start, world.goal)
    final_map.save(output_dir / "navigation_map.png")
    if frames:
        frames[::2][0].save(output_dir / "navigation_run.gif", save_all=True,
                            append_images=frames[::2][1:], duration=100, loop=0, optimize=True)
    result: dict[str, int | float | bool] = {
        "goal_reached": estimated_pose == world.goal,
        "navigation_steps": len(visited) - 1,
        "planned_route_length_cells": max(0, len(route) - 1),
        "map_coverage": round(grid.coverage(), 4),
        "final_x_cell": estimated_pose[0],
        "final_y_cell": estimated_pose[1],
    }
    (output_dir / "navigation_metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return result
