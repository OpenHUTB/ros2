"""到目标的测地距离图：给策略一个“往哪走更近”的局部代价信号。

没有它，同样的局部观测可能对应相反的专家动作（绕左边还是绕右边取决于
全局路径），网络学不动；有了它，最优动作就是沿距离下降方向走。
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .grid import RES, cell_center, to_cell

NEIGHBOURS = ((-1, -1), (-1, 0), (-1, 1), (0, -1),
              (0, 1), (1, -1), (1, 0), (1, 1))

__all__ = ["distance_map", "descend_path"]


def distance_map(g: np.ndarray, goal_xy) -> np.ndarray:
    """Dijkstra：每个空闲格到目标的测地距离（米），障碍为 inf。"""
    h, w = g.shape
    dist = np.full((h, w), np.inf, np.float64)
    gi, gj = to_cell(g, goal_xy[0], goal_xy[1])
    if g[gj, gi]:
        return dist

    dist[gj, gi] = 0.0
    heap = [(0.0, gi, gj)]
    while heap:
        d, ci, cj = heapq.heappop(heap)
        if d > dist[cj, ci]:
            continue
        for di, dj in NEIGHBOURS:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < w and 0 <= nj < h) or g[nj, ni]:
                continue
            if di and dj and (g[cj, ni] or g[nj, ci]):   # 不许贴角穿过
                continue
            nd = d + math.hypot(di, dj) * RES
            if nd < dist[nj, ni] - 1e-9:
                dist[nj, ni] = nd
                heapq.heappush(heap, (nd, ni, nj))
    return dist


def descend_path(dist: np.ndarray, g: np.ndarray, start_xy,
                 max_steps: int = 2000):
    """沿距离图下降，得到从当前位置到目标的路径（世界坐标）。"""
    h, w = dist.shape
    ci, cj = to_cell(g, start_xy[0], start_xy[1])
    if not np.isfinite(dist[cj, ci]):
        return None

    path = []
    seen = set()
    for _ in range(max_steps):
        path.append(cell_center(ci, cj))
        if dist[cj, ci] <= 0.0:
            return path
        if (ci, cj) in seen:
            return None
        seen.add((ci, cj))

        best, bi, bj = float("inf"), None, None
        for di, dj in NEIGHBOURS:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < w and 0 <= nj < h):
                continue
            if di and dj and (g[cj, ni] or g[nj, ci]):
                continue
            if dist[nj, ni] < best:
                best, bi, bj = dist[nj, ni], ni, nj
        if bi is None or not np.isfinite(best):
            return None
        ci, cj = bi, bj
    return None
