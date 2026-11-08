"""八邻域 A*：不可贴角穿过，边界用 floor + eps 修正。"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .grid import RES, is_free, to_cell

NEIGHBOURS = ((-1, -1), (-1, 0), (-1, 1), (0, -1),
              (0, 1), (1, -1), (1, 0), (1, 1))

__all__ = ["astar"]


def astar(g: np.ndarray, start, goal):
    """起点终点为世界坐标；返回世界坐标列表，不可达返回 None。"""
    h, w = g.shape
    si, sj = to_cell(g, *start)
    gi, gj = to_cell(g, *goal)
    if g[sj, si] or g[gj, gi]:
        return None

    s = sj * w + si
    t = gj * w + gi

    best = {s: 0.0}
    came: dict = {}
    heap = [(0.0, s)]
    closed = set()

    while heap:
        _, cur = heapq.heappop(heap)
        if cur == t:
            cells = [cur]
            while cur in came:
                cur = came[cur]
                cells.append(cur)
            cells.reverse()
            return [((c % w + 0.5) * RES, (c // w + 0.5) * RES) for c in cells]
        if cur in closed:
            continue
        closed.add(cur)

        ci, cj = cur % w, cur // w
        for di, dj in NEIGHBOURS:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < w and 0 <= nj < h):
                continue
            n = nj * w + ni
            if g[nj, ni]:
                continue
            if di and dj:                      # 对角不许贴角穿过
                if g[cj, ni] or g[nj, ci]:
                    continue
            cand = best[cur] + math.hypot(di, dj)
            if cand < best.get(n, float("inf")):
                best[n] = cand
                came[n] = cur
                heapq.heappush(
                    heap, (cand + math.hypot(gi - ni, gj - nj), n))
    return None
