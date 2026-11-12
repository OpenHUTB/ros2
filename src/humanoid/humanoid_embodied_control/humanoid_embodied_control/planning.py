# -*- coding: utf-8 -*-
"""决策层：占据栅格 + A* 全局路径 + 纯跟踪前视点。

人形机器人不靠"看见墙就转一下"，而是先在地图上规划一条能走的路，
再用短视距的纯跟踪去跟。这样两个好处：
* 决策层只需要给"下一个前视点在哪"，步态层不必知道地图；
* 感知层更新地图后，重规划只花毫秒级时间，不需要重训练任何东西。

栅格约定：世界系 x 向东为列、y 向北为行，分辨率 ``res`` 米/格，原点为
``(x0, y0)`` 所在格的中心。障碍物以"半径膨胀"的方式写入（把机器人半宽
算进障碍里），这样 A* 找到的路径天然带安全余量。

实现都写成纯函数 / 纯数据结构，不依赖 numpy 与 ROS，便于单测。
"""

from __future__ import annotations

import heapq
import math

from .geometry import wrap_pi


class CostMap:
    """二值占据栅格（1=占据）。"""

    def __init__(self, width: int, height: int, res: float, x0: float = 0.0, y0: float = 0.0):
        if width <= 0 or height <= 0:
            raise ValueError("栅格尺寸必须为正")
        if res <= 0.0:
            raise ValueError("分辨率必须为正")
        self.width = int(width)
        self.height = int(height)
        self.res = float(res)
        self.x0 = float(x0)
        self.y0 = float(y0)
        self.grid = [[0] * self.width for _ in range(self.height)]

    # ---------- 坐标换算 ----------
    def world_to_cell(self, x: float, y: float):
        col = int(math.floor((x - self.x0) / self.res))
        row = int(math.floor((y - self.y0) / self.res))
        return row, col

    def cell_to_world(self, row: int, col: int):
        x = self.x0 + (col + 0.5) * self.res
        y = self.y0 + (row + 0.5) * self.res
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    # ---------- 写入 ----------
    def set_occupied(self, row: int, col: int) -> None:
        if self.in_bounds(row, col):
            self.grid[row][col] = 1

    def add_circle(self, x: float, y: float, radius: float) -> None:
        """把半径 ``radius``（米）的圆形障碍写进栅格。

        用「距离平方比较」而不是开方：批量写障碍时开方是热点，
        而且平方比较对浮点比较更稳定。
        """
        if radius <= 0.0:
            return
        r2 = radius * radius
        col_lo, col_hi = self.world_to_cell(x - radius, y - radius)[1], self.world_to_cell(x + radius, y + radius)[1]
        row_lo, row_hi = self.world_to_cell(x - radius, y - radius)[0], self.world_to_cell(x + radius, y + radius)[0]
        for row in range(max(0, row_lo), min(self.height - 1, row_hi) + 1):
            for col in range(max(0, col_lo), min(self.width - 1, col_hi) + 1):
                cx, cy = self.cell_to_world(row, col)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r2:
                    self.grid[row][col] = 1

    def add_obstacle_runs(self, runs, robot_radius: float) -> None:
        """把感知层给出的障碍段写进地图。

        ``runs`` 每项含 ``angle``（相对角）、``distance``（米）、``width``（扇区数）。
        这里把障碍近似成「在极坐标上的一段圆弧」：沿障碍方向按距离放置若干
        点，每个点写半径为 ``robot_radius`` 的圆。圆弧长度按扇区宽度估算，
        弧长不足一个格子时至少放一个点，避免近处细柱子被漏掉。
        """
        for run in runs:
            ang = float(run["angle"])
            dist = float(run["distance"])
            width = max(1, int(run.get("width", 1)))
            if dist <= 0.0 or dist > 1e6:
                continue
            arc = max(self.res, dist * 0.05)
            steps = max(1, min(9, int(round(width * arc / self.res))))
            for k in range(steps):
                a = ang if steps == 1 else ang + (k / (steps - 1) - 0.5) * 0.12 * width
                ox = dist * math.cos(a)
                oy = dist * math.sin(a)
                self.add_circle(ox, oy, robot_radius)

    # ---------- 查询 ----------
    def is_free(self, row: int, col: int) -> bool:
        return self.in_bounds(row, col) and self.grid[row][col] == 0

    def occupied_ratio(self) -> float:
        total = self.width * self.height
        if total == 0:
            return 0.0
        occ = sum(sum(row) for row in self.grid)
        return occ / total

    def clearance(self, x: float, y: float, radius: float) -> float:
        """返回 (x,y) 附近 ``radius`` 内是否无障碍：1=完全空，0=有障碍。

        用采样近似而不是精确距离变换——规划只需要一个"能不能过"的判定，
        精确 EDT 会引入不必要的复杂度。
        """
        if radius <= 0.0:
            return 1.0
        row, col = self.world_to_cell(x, y)
        if not self.is_free(row, col):
            return 0.0
        rad_cells = int(math.ceil(radius / self.res))
        for dr in range(-rad_cells, rad_cells + 1):
            for dc in range(-rad_cells, rad_cells + 1):
                if dr == 0 and dc == 0:
                    continue
                if (dr * self.res) ** 2 + (dc * self.res) ** 2 > radius * radius:
                    continue
                if not self.is_free(row + dr, col + dc):
                    return 0.0
        return 1.0


_HEURISTIC = math.sqrt(2.0)

_NEIGHBOURS = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _HEURISTIC), (-1, 1, _HEURISTIC), (1, -1, _HEURISTIC), (1, 1, _HEURISTIC),
)


def astar(cost_map: CostMap, start, goal):
    """八邻域 A*，返回格坐标路径 ``[(row, col), ...]``；不可达返回 ``[]``。

    对角线移动要求两个相邻正交格都可通行（即"不切角"）：只有对角线可走时
    人形的肩宽会剐到墙角，实际步行会卡住，因此必须在规划阶段就禁掉。

    用 ``heapq`` + ``g`` 字典的惰性删除实现；不写 closed 集合是为了避免
    "发现更短路径却因为已在 closed 中被丢弃"的经典错误。
    """
    start = tuple(start)
    goal = tuple(goal)
    if not cost_map.is_free(*start) or not cost_map.is_free(*goal):
        return []
    if start == goal:
        return [start]

    open_heap = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cur_g = g_score.get(current, float("inf"))
        for dr, dc, step in _NEIGHBOURS:
            nb = (current[0] + dr, current[1] + dc)
            if not cost_map.is_free(*nb):
                continue
            if dr != 0 and dc != 0:
                if not cost_map.is_free(current[0] + dr, current[1]) or \
                   not cost_map.is_free(current[0], current[1] + dc):
                    continue
            tentative = cur_g + step
            if tentative < g_score.get(nb, float("inf")) - 1e-12:
                g_score[nb] = tentative
                came_from[nb] = current
                f = tentative + _HEURISTIC * math.hypot(nb[0] - goal[0], nb[1] - goal[1])
                heapq.heappush(open_heap, (f, nb))
    return []


def path_to_world(cost_map: CostMap, path):
    """格坐标路径 → 世界坐标点列。"""
    return [cost_map.cell_to_world(r, c) for r, c in path]


def path_length(points) -> float:
    """折线总长度（米）。"""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def lookahead_point(path, x: float, y: float, lookahead: float):
    """沿路径找出第一个距当前位置至少 ``lookahead`` 的点。

    为什么不用"最近路点"：路点稀疏时最近点可能已经落在身后，
    纯跟踪会朝身后追，表现为原地摆动。用前视距离则天然向前看。
    若整条路径都在前视距离以内，返回最后一个点（接近终点时自然收敛）。
    """
    if not path:
        return None
    if lookahead <= 0.0:
        return path[0]
    for px, py in path:
        if math.hypot(px - x, py - y) >= lookahead:
            return px, py
    return path[-1]


def heading_error(x: float, y: float, yaw: float, target) -> float:
    """目标方位相对当前航向的差角（弧度，逆时针为正，已归一）。"""
    dx = target[0] - x
    dy = target[1] - y
    return wrap_pi(math.atan2(dy, dx) - yaw)


def replan_needed(heading_err: float, obstacle_dist: float, front_clear: float) -> bool:
    """是否需要重新规划：航向偏差过大 / 正前方过近。

    阈值取 0.6 rad（≈34°）与 0.45 m：前者小于它时纯跟踪还能自己修正，
    后者是人形保持步行所需的最小落脚空间。
    """
    return abs(heading_err) > 0.6 or front_clear < obstacle_dist * 0.45
