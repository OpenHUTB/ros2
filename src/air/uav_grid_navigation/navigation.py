"""在预定义二维占据栅格上执行 A* 路径规划。"""

from __future__ import annotations

import heapq
import json
from pathlib import Path
from typing import Sequence

Cell = tuple[int, int]
Grid = Sequence[Sequence[int]]


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
    if grid[start[1]][start[0]] or grid[goal[1]][goal[0]]:
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
            if not inside(neighbor) or grid[neighbor[1]][neighbor[0]]:
                continue
            next_cost = cost[current] + 1
            if neighbor in cost and next_cost >= cost[neighbor]:
                continue
            cost[neighbor] = next_cost
            parent[neighbor] = current
            heuristic = abs(goal[0] - neighbor[0]) + abs(goal[1] - neighbor[1])
            order += 1
            heapq.heappush(frontier, (next_cost + heuristic, order, neighbor))
    return []


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


def main(output: Path = Path("output/grid_navigation.json")) -> None:
    grid, start, goal = demo_grid()
    result = simulate_route(grid, start, goal)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
