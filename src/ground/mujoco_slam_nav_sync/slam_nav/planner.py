"""栅格 A* 全局规划 + 纯追踪（用于给神经网络当"专家"）。"""

from __future__ import annotations

import heapq

import numpy as np

from .env import wrap
from .slam import GRID_H, GRID_RES, GRID_W, GRID_X0, GRID_Y0

ORIGIN_X, ORIGIN_Y = GRID_X0, GRID_Y0


def world_to_cell(x: float, y: float):
    return int((x - ORIGIN_X) / GRID_RES), int((y - ORIGIN_Y) / GRID_RES)


def cell_to_world(cx: int, cy: int):
    return (cx * GRID_RES + ORIGIN_X + GRID_RES / 2,
            cy * GRID_RES + ORIGIN_Y + GRID_RES / 2)


def _nearest_free(occ: np.ndarray, cx: int, cy: int, max_r: int = 12):
    h, w = occ.shape
    if 0 <= cx < w and 0 <= cy < h and not occ[cy, cx]:
        return cx, cy
    for r in range(1, max_r + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and not occ[ny, nx]:
                    return nx, ny
    return None


def astar(occ: np.ndarray, start_xy, goal_xy, max_expand: int = 60000):
    """8 邻域 A*，返回世界坐标航点列表；无解返回 None。"""
    sx, sy = _nearest_free(occ, *world_to_cell(*start_xy)) or (None, None)
    gx, gy = _nearest_free(occ, *world_to_cell(*goal_xy)) or (None, None)
    if sx is None or gx is None:
        return None
    h, w = occ.shape
    start = (sx, sy)
    goal = (gx, gy)
    if start == goal:
        return [cell_to_world(*goal)]

    open_heap = [(0.0, 0.0, start)]
    came = {}
    gscore = {start: 0.0}
    seen = set()
    expanded = 0
    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in seen:
            continue
        seen.add(cur)
        expanded += 1
        if expanded > max_expand:
            return None
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return [cell_to_world(*c) for c in path]
        cx, cy = cur
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or occ[ny, nx]:
                continue
            if dx and dy and (occ[cy, nx] or occ[ny, cx]):   # 禁止斜穿墙角
                continue
            step = 1.4142 if (dx and dy) else 1.0
            ng = gscore[cur] + step
            if ng < gscore.get((nx, ny), 1e18):
                gscore[(nx, ny)] = ng
                hh = np.hypot(nx - goal[0], ny - goal[1])
                came[(nx, ny)] = cur
                heapq.heappush(open_heap, (ng + hh, ng, (nx, ny)))
    return None


def lookahead_point(path, pose, ld: float = 0.9):
    """沿路径取距离机器人 ld 米的前视点；路径末端则返回终点。"""
    if not path:
        return None
    x, y, _ = pose
    for px, py in path:
        if np.hypot(px - x, py - y) >= ld:
            return px, py
    return path[-1]


def lookahead_points(path, pose, ld: float = 0.9, n: int = 3):
    """取 ld, 2ld, 3ld 三个前视点（神经网络需要"提前看到弯"）。"""
    if not path:
        return []
    x, y, _ = pose
    pts, d = [], ld
    for _ in range(n):
        px, py = path[-1]
        for wx, wy in path:
            if np.hypot(wx - x, wy - y) >= d:
                px, py = wx, wy
                break
        pts.append((px, py))
        d += ld
    return pts


def pure_pursuit(pose, target, v_max: float = 0.8, w_max: float = 1.5,
                 k_heading: float = 1.8, goal_tol: float = 0.35):
    """返回 (v, w)：朝前视点转，接近目标点时减速。"""
    x, y, yaw = pose
    dx, dy = target[0] - x, target[1] - y
    dist = float(np.hypot(dx, dy))
    heading = wrap(float(np.arctan2(dy, dx)) - yaw)
    w = float(np.clip(k_heading * heading, -w_max, w_max))
    v = v_max * max(0.15, 1.0 - abs(heading) / 1.2) * min(1.0, dist / 1.2 + 0.3)
    if dist < goal_tol:
        v = 0.0
    return float(v), w


def clearance(scan: np.ndarray) -> float:
    return float(np.min(scan))
