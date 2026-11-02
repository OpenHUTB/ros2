#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语义代价图上的 A* 规划，以及路径后处理。

代价定义在格上：``cost[r, c]`` 是从该格**穿过**的代价倍率，``inf`` 表示
不可通行。8 邻域的边权取两端格代价的均值乘以步长，这样路径会自然偏向
低代价格（地面、门），绕开高代价格（植被）与不可通行格（建筑、障碍）。

同一份代码既跑几何代价图（只有占据/空闲）也跑语义代价图，两者的唯一差别
是喂进来的 ``cost`` 数组，对比实验因此是受控的。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = ["PlanConfig", "PlanResult", "astar", "geometric_cost",
           "smooth_path", "path_length", "path_points"]

_MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
          (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


@dataclass
class PlanConfig:
    #: 未知区域的额外代价（鼓励走已观测区域，但不禁止探索）
    unknown_penalty: float = 0.0
    #: 语义不确定性的惩罚系数
    uncertainty_penalty: float = 0.0
    #: 到目标的容差（格）
    tol: int = 1
    max_expand: int = 400000


@dataclass
class PlanResult:
    ok: bool
    cells: List[Tuple[int, int]]
    cost: float
    expanded: int
    reason: str = ""

    def xy(self, grid) -> np.ndarray:
        return np.asarray([grid.centre(r, c) for r, c in self.cells], float) \
            if self.cells else np.zeros((0, 2))


# --------------------------------------------------------------------------- #
def geometric_cost(grid) -> np.ndarray:
    """几何代价图：只看占据概率，不看类别。作为语义代价图的基线。"""
    p = grid.probability()
    obs = grid.observed()
    cost = np.ones_like(p)
    cost[~obs] = 2.0                 # 未观测：中性
    cost[p > 0.7] = np.inf           # 占据：不可通行
    return cost


def astar(cost: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int],
          cfg: Optional[PlanConfig] = None,
          extra: Optional[np.ndarray] = None) -> PlanResult:
    """8 邻域 A*。``extra`` 是叠加在每个格上的额外代价（不确定性等）。

    返回的 ``cells`` 含起点与终点，按 (row, col) 排列。
    """
    cfg = cfg or PlanConfig()
    n = cost.shape[0]
    if cost.shape[0] != cost.shape[1]:
        raise ValueError("代价图必须是方阵")

    def blocked(rc):
        r, c = rc
        if not (0 <= r < n and 0 <= c < n):
            return True
        v = cost[r, c]
        return (not np.isfinite(v)) or v == np.inf

    if blocked(start) or blocked(goal):
        return PlanResult(False, [], float("inf"), 0, "起点或终点不可通行")

    def h(rc):
        return math.hypot(rc[0] - goal[0], rc[1] - goal[1])

    ex = extra if extra is not None else np.zeros_like(cost)
    openq = [(h(start), 0.0, start)]
    seen = {start: 0.0}
    parent = {start: None}
    expanded = 0
    while openq:
        _, g, cur = heapq.heappop(openq)
        if cur == goal or (abs(cur[0] - goal[0]) <= cfg.tol
                           and abs(cur[1] - goal[1]) <= cfg.tol):
            cells = []
            k = cur
            while k is not None:
                cells.append(k)
                k = parent[k]
            cells.reverse()
            return PlanResult(True, cells, g, expanded)
        if g > seen.get(cur, float("inf")):
            continue
        expanded += 1
        if expanded > cfg.max_expand:
            return PlanResult(False, [], float("inf"), expanded, "超出展开上限")
        for dr, dc, w in _MOVES:
            nxt = (cur[0] + dr, cur[1] + dc)
            if blocked(nxt):
                continue
            a, b = cost[cur], cost[nxt]
            if not (np.isfinite(a) and np.isfinite(b)):
                continue
            e = w * 0.5 * (a + b)
            e += w * 0.5 * (ex[cur] + ex[nxt])
            ng = g + e
            if ng < seen.get(nxt, float("inf")) - 1e-12:
                seen[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(openq, (ng + h(nxt), ng, nxt))
    return PlanResult(False, [], float("inf"), expanded, "无可行路径")


# --------------------------------------------------------------------------- #
def _line_ok(cost: np.ndarray, a: Tuple[int, int], b: Tuple[int, int],
             n_sample: int = 40) -> bool:
    t = np.linspace(0.0, 1.0, n_sample)
    rr = np.rint(a[0] + t * (b[0] - a[0])).astype(int)
    cc = np.rint(a[1] + t * (b[1] - a[1])).astype(int)
    n = cost.shape[0]
    ok = (rr >= 0) & (rr < n) & (cc >= 0) & (cc < n)
    if not np.all(ok):
        return False
    v = cost[rr, cc]
    return bool(np.all(np.isfinite(v)))


def smooth_path(cost: np.ndarray, cells: Sequence[Tuple[int, int]]
                ) -> List[Tuple[int, int]]:
    """视线捷径：能直线通达就跳过中间点。栅格 A* 的锯齿靠这一步去掉。"""
    if len(cells) <= 2:
        return list(cells)
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_ok(cost, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


def path_length(grid, cells: Sequence[Tuple[int, int]]) -> float:
    if len(cells) < 2:
        return 0.0
    p = np.asarray([grid.centre(r, c) for r, c in cells], float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def path_points(grid, cells: Sequence[Tuple[int, int]]) -> np.ndarray:
    if not len(cells):
        return np.zeros((0, 2))
    return np.asarray([grid.centre(r, c) for r, c in cells], float)
