"""在预定义二维占据栅格上执行 A* 路径规划。"""

from __future__ import annotations

import heapq
import json
from pathlib import Path
from typing import Sequence

from PIL import Image, ImageDraw

Cell = tuple[int, int]
Grid = Sequence[Sequence[int]]


def bresenham(start: Cell, end: Cell) -> list[Cell]:
    """List grid cells crossed by a line between two endpoints."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx - dy
    cells: list[Cell] = []
    while True:
        cells.append((x0, y0))
        if (x0, y0) == (x1, y1):
            return cells
        doubled_error = 2 * error
        if doubled_error > -dy:
            error -= dy
            x0 += step_x
        if doubled_error < dx:
            error += dx
            y0 += step_y


def plan_route(grid: Grid, start: Cell, goal: Cell) -> list[Cell]:
    """返回四邻域 A* 路线；目标被占用或不可达时返回空列表。"""
    if not grid or not grid[0]:
        raise ValueError("grid must have at least one row and one column")
    width = len(grid[0])
    if any(len(row) != width for row in grid):
        raise ValueError("grid rows must have equal width")

    height = len(grid)

    def inside(cell: Cell) -> bool:
        return 0 <= cell[0] < width and 0 <= cell[1] < height

    if not inside(start) or not inside(goal):
        raise ValueError("start and goal must be inside the grid")
    if grid[start[1]][start[0]] == 1 or grid[goal[1]][goal[0]] == 1:
        return []

    frontier: list[tuple[int, int, Cell]] = [(0, 0, start)]
    parent: dict[Cell, Cell | None] = {start: None}
    cost = {start: 0}
    order = 0
    while frontier:
        _priority, _order, current = heapq.heappop(frontier)
        if current == goal:
            route = [current]
            while parent[route[-1]] is not None:
                route.append(parent[route[-1]])
            return list(reversed(route))

        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if not inside(neighbor) or grid[neighbor[1]][neighbor[0]] == 1:
                continue
            next_cost = cost[current] + (4 if grid[neighbor[1]][neighbor[0]] < 0 else 1)
            if neighbor in cost and next_cost >= cost[neighbor]:
                continue
            cost[neighbor] = next_cost
            parent[neighbor] = current
            heuristic = abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1])
            order += 1
            heapq.heappush(frontier, (next_cost + heuristic, order, neighbor))
    return []


def mark_blocked_waypoint(grid: list[list[int]], cell: Cell, contact_detected: bool) -> bool:
    """Mark a waypoint occupied after a detected collision and request replanning."""
    if not contact_detected:
        return False
    x, y = cell
    if not 0 <= y < len(grid) or not 0 <= x < len(grid[y]):
        raise ValueError("blocked waypoint must be inside the occupancy grid")
    grid[y][x] = 1
    return True


def simulate_route(grid: Grid, start: Cell, goal: Cell) -> dict[str, object]:
    """每步沿离散路线前进一步，并返回路线指标。"""
    route = plan_route(grid, start, goal)
    final_cell = route[-1] if route else start
    return {
        "goal_reached": bool(route and final_cell == goal),
        "step_count": max(0, len(route) - 1),
        "route": [list(cell) for cell in route],
        "final_cell": list(final_cell),
    }


def demo_grid() -> tuple[list[list[int]], Cell, Cell]:
    """构建一道障碍墙及其单一通道组成的小型占据栅格。"""
    grid = [[0 for _ in range(12)] for _ in range(8)]
    for y in range(1, 7):
        grid[y][5] = 1
    grid[4][5] = 0
    return grid, (1, 1), (10, 6)


def render_route(
    grid: Grid,
    route: Sequence[Cell],
    start: Cell,
    goal: Cell,
    output: Path | None = None,
    cell_size: int = 48,
    visited: Sequence[Cell] = (),
) -> Image.Image:
    """将占据栅格、规划路线、起点和终点渲染为 PNG。"""
    if cell_size < 12:
        raise ValueError("cell_size must be at least 12 pixels")
    height, width = len(grid), len(grid[0])
    margin = 18
    header_height = 34
    footer_height = 34
    grid_left = margin
    grid_top = header_height
    image = Image.new(
        "RGB",
        (width * cell_size + 2 * margin, height * cell_size + header_height + footer_height),
        "#ffffff",
    )
    draw = ImageDraw.Draw(image)
    route_cells = set(route)
    step_count = max(0, len(visited) - 1) if visited else max(0, len(route) - 1)
    draw.text(
        (margin, 9),
        f"UAV A* ROUTE | {step_count} STEPS | {'GOAL REACHED' if route and route[-1] == goal else 'NO ROUTE'}",
        fill="#26383d",
    )
    for y, row in enumerate(grid):
        for x, occupied in enumerate(row):
            left = grid_left + x * cell_size
            top = grid_top + (height - y - 1) * cell_size
            right, bottom = left + cell_size - 1, top + cell_size - 1
            color = (
                "#34444a" if occupied == 1 else
                "#bdc5c8" if occupied < 0 else
                "#5fb6a9" if (x, y) in route_cells else "#f4f6f5"
            )
            draw.rectangle((left, top, right, bottom), fill=color, outline="#c7d1d0", width=1)

    for x, y in visited:
        center_x = grid_left + x * cell_size + cell_size // 2
        center_y = grid_top + (height - y - 1) * cell_size + cell_size // 2
        radius = max(2, cell_size // 6)
        draw.ellipse((center_x - radius, center_y - radius, center_x + radius, center_y + radius), fill="#3c9b61")

    radius = max(4, cell_size // 4)
    for cell, color in ((start, "#2468a2"), (goal, "#d4674f")):
        x, y = cell
        center_x = grid_left + x * cell_size + cell_size // 2
        center_y = grid_top + (height - y - 1) * cell_size + cell_size // 2
        draw.ellipse(
            (center_x - radius, center_y - radius, center_x + radius, center_y + radius),
            fill=color,
            outline="white",
            width=2,
        )
    legend_y = grid_top + height * cell_size + 11
    legend = (("Route", "#5fb6a9"), ("Obstacle", "#34444a"), ("Start", "#2468a2"), ("Goal", "#d4674f"))
    for index, (label, color) in enumerate(legend):
        left = margin + index * 112
        draw.rectangle((left, legend_y, left + 11, legend_y + 11), fill=color)
        draw.text((left + 16, legend_y + 1), label, fill="#26383d")
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        image.save(output, format="PNG")
    return image


def main(output_dir: Path = Path("output")) -> None:
    if __package__:
        from .simulation import run_mujoco_navigation
    else:
        from simulation import run_mujoco_navigation

    result = run_mujoco_navigation(output_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
