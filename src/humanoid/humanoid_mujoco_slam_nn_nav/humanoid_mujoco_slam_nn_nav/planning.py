"""全局路径规划（A*）与路径跟踪（纯跟踪 / Pure Pursuit）。

规划在"膨胀后的占据栅格"上进行：机器人半径按 ``inflation`` 膨胀障碍物，
保证 A* 出来的路径对双足也留有余量（脚掌外沿）。
"""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]


class CostMap:
    """把 OccupancyGrid 转成规划用的可通行栅格（含膨胀）。"""

    def __init__(self, grid, inflation: float = 0.28, occ_thresh: float = 0.62):
        self.grid = grid
        occ = grid.occupied(occ_thresh)
        self.blocked = self._inflate(occ, inflation)
        self.res = grid.res
        self.origin = grid.origin

    def _inflate(self, occ: np.ndarray, radius: float) -> np.ndarray:
        r = max(1, int(round(radius / self.grid.res)))
        out = occ.copy()
        if r <= 0:
            return out
        # 用可分离的方形结构元素做膨胀，等价于切比雪夫距离
        src = occ.copy()
        tmp = src.copy()
        for dy in range(-r, r + 1):
            tmp |= np.roll(src, dy, axis=0)
        for dx in range(-r, r + 1):
            out |= np.roll(tmp, dx, axis=1)
        return out

    def is_free(self, cx: int, cy: int) -> bool:
        if cx < 0 or cy < 0 or cx >= self.grid.nx or cy >= self.grid.ny:
            return False
        return not self.blocked[cy, cx]


def astar(costmap: CostMap, start_xy, goal_xy, max_iter: int = 200000) -> List[Tuple[float, float]]:
    """8 邻域 A*，返回世界坐标路径（不含起点）或空列表。"""
    g = costmap.grid
    sx, sy = g.world_to_cell(*start_xy)
    gx, gy = g.world_to_cell(*goal_xy)
    if not costmap.is_free(int(gx), int(gy)):
        gx, gy = _nearest_free(costmap, int(gx), int(gy))
        if gx is None:
            return []
    if not costmap.is_free(int(sx), int(sy)):
        nsx, nsy = _nearest_free(costmap, int(sx), int(sy))
        if nsx is None:
            return []
        sx, sy = nsx, nsy
    start = (int(sx), int(sy))
    goal = (int(gx), int(gy))
    if start == goal:
        return [tuple(goal_xy)]

    open_heap = [(0.0, 0.0, start)]
    came = {}
    gscore = {start: 0.0}
    it = 0
    while open_heap and it < max_iter:
        it += 1
        _, gc, cur = heapq.heappop(open_heap)
        if cur == goal:
            return _reconstruct(came, cur, g)
        if gc > gscore.get(cur, 1e9):
            continue
        for dx, dy in NEIGHBORS:
            nb = (cur[0] + dx, cur[1] + dy)
            if not costmap.is_free(*nb):
                continue
            step = math.hypot(dx, dy)
            tentative = gc + step
            if tentative < gscore.get(nb, 1e9):
                gscore[nb] = tentative
                h = math.hypot(nb[0] - goal[0], nb[1] - goal[1])
                came[nb] = cur
                heapq.heappush(open_heap, (tentative + h, tentative, nb))
    return []


def _nearest_free(costmap: CostMap, cx: int, cy: int, radius: int = 12):
    best = None
    best_d = 1e9
    for r in range(1, radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                x, y = cx + dx, cy + dy
                if costmap.is_free(x, y):
                    d = math.hypot(dx, dy)
                    if d < best_d:
                        best_d, best = d, (x, y)
        if best:
            return best
    return None, None


def _reconstruct(came, cur, grid):
    cells = [cur]
    while cur in came:
        cur = came[cur]
        cells.append(cur)
    cells.reverse()
    pts = []
    for cx, cy in cells[1:]:
        x, y = grid.cell_to_world(cx, cy)
        pts.append((float(x), float(y)))
    return pts


def simplify(path: Sequence[Tuple[float, float]], eps: float = 0.05) -> List[Tuple[float, float]]:
    """去掉共线点，减少路径点数。"""
    if len(path) < 3:
        return list(path)
    out = [tuple(path[0])]
    for i in range(1, len(path) - 1):
        x0, y0 = out[-1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if abs(cross) > eps:
            out.append((x1, y1))
    out.append(tuple(path[-1]))
    return out


class PurePursuit:
    """纯跟踪控制器：输出 (v, ω)。"""

    def __init__(self, lookahead: float = 0.55, v_max: float = 0.28,
                 v_min: float = 0.08, w_max: float = 0.7):
        self.lookahead = lookahead
        self.v_max = v_max
        self.v_min = v_min
        self.w_max = w_max

    def lookahead_point(self, pose, path, idx: int = 0):
        """返回 (目标点, 新的最近点索引)。"""
        x, y, yaw = pose
        if not path:
            return None, idx
        best_i = idx
        best_d = 1e9
        for i in range(idx, min(len(path), idx + 40)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d < best_d:
                best_d, best_i = d, i
        for i in range(best_i, len(path)):
            if math.hypot(path[i][0] - x, path[i][1] - y) >= self.lookahead:
                return path[i], best_i
        return path[-1], best_i

    def control(self, pose, path, idx: int = 0, goal_xy=None):
        x, y, yaw = pose
        if not path:
            return 0.0, 0.0, idx
        target, best_i = self.lookahead_point(pose, path, idx)
        dx = target[0] - x
        dy = target[1] - y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        w = float(np.clip(1.6 * err, -self.w_max, self.w_max))
        v = self.v_max * max(0.0, 1.0 - abs(err) / 1.2)
        if best_i >= len(path) - 3:
            v = min(v, max(self.v_min, 0.6 * dist))
        v = float(np.clip(v, 0.0, self.v_max))
        return v, w, best_i
