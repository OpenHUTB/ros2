"""A* and Dijkstra solvers with a pluggable (neural) heuristic."""

from __future__ import annotations

import heapq
import math
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


def nearest_free(occ: np.ndarray, cell: Tuple[int, int], radius: int = 6) -> Optional[Tuple[int, int]]:
    nx, ny = occ.shape
    x, y = cell
    if 0 <= x < nx and 0 <= y < ny and not occ[x, y]:
        return x, y
    best, bd = None, 1e9
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            gx, gy = x + dx, y + dy
            if 0 <= gx < nx and 0 <= gy < ny and not occ[gx, gy]:
                d = dx * dx + dy * dy
                if d < bd:
                    bd, best = d, (gx, gy)
    return best


def dijkstra(occ: np.ndarray, goal: Tuple[int, int], res: float = 0.5) -> np.ndarray:
    """Cost-to-go (metres) from every free cell to ``goal``; inf if unreachable."""
    nx, ny = occ.shape
    dist = np.full((nx, ny), np.inf, dtype=np.float64)
    gx, gy = goal
    if not (0 <= gx < nx and 0 <= gy < ny) or occ[gx, gy]:
        return dist
    dist[gx, gy] = 0.0
    pq = [(0.0, gx, gy)]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[x, y] + 1e-9:
            continue
        for dx, dy, c in NEIGHBOURS:
            nx_, ny_ = x + dx, y + dy
            if not (0 <= nx_ < nx and 0 <= ny_ < ny) or occ[nx_, ny_]:
                continue
            if dx and dy and (occ[x + dx, y] or occ[x, y + dy]):
                continue
            nd = d + c * res
            if nd < dist[nx_, ny_] - 1e-9:
                dist[nx_, ny_] = nd
                heapq.heappush(pq, (nd, nx_, ny_))
    return dist


def astar(occ: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int],
          res: float = 0.5, heuristic: Optional[np.ndarray] = None,
          weight: float = 1.0, max_expand: int = 40000) -> Dict:
    """8-connected A* on a boolean occupancy grid.

    ``heuristic`` is an optional (nx, ny) cost-to-go field in metres.  The
    returned path is a list of cell centres in world coordinates.
    """
    nx, ny = occ.shape
    s = nearest_free(occ, start)
    g = nearest_free(occ, goal)
    if s is None or g is None:
        return {"success": False, "path": [], "expanded": 0, "cost": float("inf")}
    s, g = s, g
    if heuristic is not None:
        h = heuristic
    else:
        h = None

    def hcost(x, y):
        if h is not None:
            v = float(h[x, y])
            if not np.isfinite(v):
                v = 1e3
            return v
        return res * math.hypot(x - g[0], y - g[1])

    gscore = np.full((nx, ny), np.inf, dtype=np.float64)
    parent = np.full((nx, ny, 2), -1, dtype=np.int32)
    closed = np.zeros((nx, ny), dtype=bool)
    gscore[s] = 0.0
    pq = [(weight * hcost(*s), 0.0, s[0], s[1])]
    expanded = 0
    found = False
    while pq:
        f, gc, x, y = heapq.heappop(pq)
        if closed[x, y]:
            continue
        closed[x, y] = True
        expanded += 1
        if (x, y) == g:
            found = True
            break
        if expanded > max_expand:
            break
        for dx, dy, c in NEIGHBOURS:
            nx_, ny_ = x + dx, y + dy
            if not (0 <= nx_ < nx and 0 <= ny_ < ny) or occ[nx_, ny_] or closed[nx_, ny_]:
                continue
            if dx and dy and (occ[x + dx, y] or occ[x, y + dy]):
                continue
            ng = gc + c * res
            if ng < gscore[nx_, ny_] - 1e-9:
                gscore[nx_, ny_] = ng
                parent[nx_, ny_] = (x, y)
                heapq.heappush(pq, (ng + weight * hcost(nx_, ny_), ng, nx_, ny_))
    if not found:
        return {"success": False, "path": [], "expanded": expanded, "cost": float("inf")}
    cells = [g]
    cur = g
    while cur != s:
        px, py = parent[cur]
        if px < 0:
            break
        cur = (int(px), int(py))
        cells.append(cur)
    cells.reverse()
    world = [(0, 0)] * len(cells)
    for i, (x, y) in enumerate(cells):
        world[i] = (x, y)
    return {"success": True, "path_cells": cells, "expanded": expanded,
            "cost": float(gscore[g])}


def line_of_sight(occ: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        if occ[x, y]:
            return False
        if (x, y) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def simplify_path(occ: np.ndarray, cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not line_of_sight(occ, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


def cells_to_world(cells, x0: float, y0: float, res: float, z: float = 3.0) -> np.ndarray:
    return np.array([[x0 + (x + 0.5) * res, y0 + (y + 0.5) * res, z] for x, y in cells],
                    dtype=float)
