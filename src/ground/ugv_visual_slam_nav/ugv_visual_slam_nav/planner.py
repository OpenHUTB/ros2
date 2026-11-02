"""栅格 A* 全局规划 + 视线简化。

规划在"占据膨胀"后的栅格上做，膨胀半径覆盖车体外接圆，
这样 A* 给出的路径本身即可通行，局部避障只需处理动态障碍。
"""

import heapq
import math

import numpy as np

from . import params as P

SQ2 = math.sqrt(2.0)
_NEIGH = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]


def inflate(mask, cells):
    """方形结构元膨胀，cells 为格数半径。"""
    m = mask.copy()
    for _ in range(int(cells)):
        nxt = m.copy()
        for dr, dc, _c in _NEIGH:
            src = np.roll(np.roll(m, dr, axis=0), dc, axis=1)
            nxt |= src
        m = nxt
    return m


def _to_cell(x, y):
    return int(math.floor(y / P.GRID_RES)), int(math.floor(x / P.GRID_RES))


def _to_world(r, c):
    return (c + 0.5) * P.GRID_RES, (r + 0.5) * P.GRID_RES


def _nearest_free(mask, r, c, radius):
    """在半径 radius 格内找最近的空格，返回 (-1,-1) 表示没有。"""
    r0, r1 = max(0, r - radius), min(P.GH, r + radius + 1)
    c0, c1 = max(0, c - radius), min(P.GW, c + radius + 1)
    sub = np.argwhere(~mask[r0:r1, c0:c1])
    if len(sub) == 0:
        return -1, -1
    d = (sub[:, 0] + r0 - r) ** 2 + (sub[:, 1] + c0 - c) ** 2
    i = int(np.argmin(d))
    return int(sub[i, 0] + r0), int(sub[i, 1] + c0)


def astar(blocked, start_xy, goal_xy, cost=None):
    """阻塞栅格上做 8 邻域 A*，返回世界坐标路径（含终点）。

    cost 为可选的逐格附加代价（例如把未探索区域设得更贵），
    用来让规划优先走已经建过图的区域。
    """
    sr, sc = _to_cell(*start_xy)
    gr, gc = _to_cell(*goal_xy)
    sr = min(max(sr, 0), P.GH - 1)
    sc = min(max(sc, 0), P.GW - 1)
    gr = min(max(gr, 0), P.GH - 1)
    gc = min(max(gc, 0), P.GW - 1)

    # 目标格可能被噪声建图误标成占据：就近换一个可通行格，而不是直接判定无解
    if blocked[gr, gc]:
        gr, gc = _nearest_free(blocked, gr, gc, 6)
        if gr < 0:
            return []
    blk = blocked.copy()
    blk[gr, gc] = False
    # 起点可能因为跟踪误差落进了膨胀区：就近找一个可通行格作为实际起点
    if blk[sr, sc]:
        sr, sc = _nearest_free(blk, sr, sc, 8)
        if sr < 0:
            return []

    cw = np.ones((P.GH, P.GW), dtype=np.float64) if cost is None else cost

    h = lambda r, c: math.hypot(r - gr, c - gc)
    gscore = np.full((P.GH, P.GW), np.inf, dtype=np.float64)
    came = np.full((P.GH, P.GW), -1, dtype=np.int64)
    closed = np.zeros((P.GH, P.GW), dtype=bool)
    gscore[sr, sc] = 0.0
    openq = [(h(sr, sc), 0.0, sr, sc)]

    while openq:
        _f, g, r, c = heapq.heappop(openq)
        if closed[r, c]:
            continue
        closed[r, c] = True
        if r == gr and c == gc:
            break
        for dr, dc, step in _NEIGH:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= P.GH or nc < 0 or nc >= P.GW:
                continue
            if closed[nr, nc] or blk[nr, nc]:
                continue
            # 斜向不允许穿墙角
            if dr and dc and (blk[r, nc] or blk[nr, c]):
                continue
            ng = g + step * cw[nr, nc]
            if ng < gscore[nr, nc]:
                gscore[nr, nc] = ng
                came[nr, nc] = r * P.GW + c
                heapq.heappush(openq, (ng + h(nr, nc), ng, nr, nc))

    if not np.isfinite(gscore[gr, gc]):
        return []

    cells = []
    r, c = gr, gc
    while r >= 0 and c >= 0:
        cells.append((r, c))
        prev = came[r, c]
        if prev < 0:
            break
        r, c = divmod(int(prev), P.GW)
    cells.reverse()
    return [_to_world(r, c) for r, c in cells]


def los_clear(blocked, p0, p1, step=None):
    """两点连线是否全程无阻塞。"""
    step = step or (P.GRID_RES * 0.5)
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(2, int(d / step) + 1)
    for i in range(n + 1):
        t = i / float(n)
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        r, c = _to_cell(x, y)
        if r < 0 or r >= P.GH or c < 0 or c >= P.GW:
            return False
        if blocked[r, c]:
            return False
    return True


def simplify(blocked, path, max_skip=40):
    """贪心视线简化：能直连就跳过中间点。"""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = min(i + max_skip, len(path) - 1)
        while j > i + 1 and not los_clear(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def proximity_cost(occ, levels=5, weight=0.9):
    """离障碍越近代价越高，把路径从"贴边最优"推向通道中线。

    A* 的最短路径天然贴着膨胀边界走，而纯追踪在转弯处会向内切，
    两者叠加就会擦到障碍。加一层距离惩罚比单纯加大膨胀更划算：
    既留出跟踪余量，又不会把窄门洞挤没。
    """
    cost = np.ones((P.GH, P.GW), dtype=np.float64)
    m = occ.copy()
    for k in range(levels):
        m = inflate(m, 1)
        cost += weight * (levels - k) / float(levels) * m
    return cost
