"""A* 栅格全局路径规划。

实现要点
--------
1. **二叉堆优先队列 + 延迟删除**：节点被再次弹出时若已收敛则跳过，
   避免 ``decrease-key`` 的额外索引维护，实测比每次线性扫描 OPEN 快一个量级。
2. **可采纳启发式**：4-邻接用曼哈顿，8-邻接用 octile（对角距离），
   二者都不超过真实代价，保证 A* 的最优性；``weight > 1`` 时退化为加权 A*，
   牺牲最优性换速度（本模块把它作为可调项而非默认值）。
3. **对角穿角检查**：8-邻接下不允许从两个占用格的夹角斜穿过去，
   否则规划出的路径会"擦着墙角走"，实际机器人会碰撞。
4. **平局打破**：``f`` 相同时优先 ``h`` 小的（更靠近目标），
   显著减少开阔地形上的无谓扩展。
5. **有界搜索**：扩展节点数超过 ``max_expansions`` 立即终止并报告，
   避免地图错误时把 CPU 打满——这是让算法能上真机的前提。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from .gridmap import GridMap

SQRT2 = math.sqrt(2.0)

# 8 邻接：(di, dj, 步长)
NEIGHBORS_8 = (
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (1, 1, SQRT2),
    (1, -1, SQRT2),
    (-1, 1, SQRT2),
    (-1, -1, SQRT2),
)

NEIGHBORS_4 = (
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 0, 1.0),
    (-1, 0, 1.0),
)


@dataclass
class PlanResult:
    """一次规划的完整结果，便于上层做诊断与出图。"""

    success: bool
    cells: list[tuple[int, int]] = field(default_factory=list)  # 栅格路径
    cost: float = 0.0            # 栅格单位下的路径代价
    expansions: int = 0          # 实际扩展的节点数
    reason: str = ""             # 失败原因（成功时为空）
    allow_diagonal: bool = True

    @property
    def n_cells(self) -> int:
        return len(self.cells)


@dataclass
class AStarConfig:
    """A* 的可调参数。"""

    allow_diagonal: bool = True      # 是否允许对角移动
    heuristic: str = "octile"        # octile | manhattan | euclidean | zero
    weight: float = 1.0              # 启发式权重，1=最优，>1 更快但次优
    max_expansions: int = 400_000    # 扩展上限，防止地图异常时死跑
    tie_break: bool = True           # f 相同时优先 h 小者

    def __post_init__(self) -> None:
        if self.heuristic not in ("octile", "manhattan", "euclidean", "zero"):
            raise ValueError(f"未知启发式: {self.heuristic}")
        if self.weight < 1.0:
            raise ValueError("weight 必须 >= 1（<1 会破坏可采纳性且无意义）")
        if self.max_expansions <= 0:
            raise ValueError("max_expansions 必须为正")


def heuristic(di: int, dj: int, heuristic_name: str, allow_diagonal: bool) -> float:
    """计算从位移 ``(di, dj)`` 到目标的启发式估计值。

    所有启发式都以"栅格步长"为单位，与 :data:`NEIGHBORS_8` 的步长一致。
    """
    adi, adj = abs(di), abs(dj)
    if heuristic_name == "zero":
        return 0.0
    if heuristic_name == "manhattan":
        return float(adi + adj)
    if heuristic_name == "euclidean":
        return math.hypot(adi, adj)
    # octile：对角步代价 sqrt2，直行步代价 1
    if allow_diagonal:
        lo, hi = (adi, adj) if adi <= adj else (adj, adi)
        return SQRT2 * lo + (hi - lo)
    return float(adi + adj)


class AStarPlanner:
    """栅格 A* 规划器。"""

    def __init__(self, config: AStarConfig | None = None) -> None:
        self.config = config or AStarConfig()

    # ---------------- 主入口 ----------------

    def plan(
        self,
        gridmap: GridMap,
        start: tuple[int, int],
        goal: tuple[int, int],
        allow_diagonal: bool | None = None,
        weight: float | None = None,
    ) -> PlanResult:
        """在 ``gridmap`` 上规划从 ``start`` 到 ``goal`` 的路径。

        Args:
            gridmap: 已按机器人半径膨胀过的地图。
            start/goal: 栅格索引 ``(i, j)``。
            allow_diagonal, weight: 覆盖配置中的同名项，便于做对照实验。

        Returns:
            :class:`PlanResult`；失败时 ``success=False`` 并给出 ``reason``。
        """
        cfg = self.config
        diag = cfg.allow_diagonal if allow_diagonal is None else allow_diagonal
        w = cfg.weight if weight is None else weight
        if w < 1.0:
            return PlanResult(False, reason=f"非法 weight={w}", allow_diagonal=diag)

        if not gridmap.in_bounds(*start):
            return PlanResult(False, reason="起点越界", allow_diagonal=diag)
        if not gridmap.in_bounds(*goal):
            return PlanResult(False, reason="终点越界", allow_diagonal=diag)
        if not gridmap.is_free(*start):
            return PlanResult(False, reason="起点位于障碍内", allow_diagonal=diag)
        if not gridmap.is_free(*goal):
            return PlanResult(False, reason="终点位于障碍内", allow_diagonal=diag)
        if start == goal:
            return PlanResult(True, [start], 0.0, 0, "", diag)

        # 连通性预检：不可达时无需跑搜索，直接给出明确原因
        if not gridmap.same_component(start, goal):
            return PlanResult(False, reason="起点与终点不连通（分属不同自由区域）", allow_diagonal=diag)

        res = self._search(gridmap, start, goal, diag, w)
        return res

    # ---------------- 搜索主体 ----------------

    def _search(
        self,
        gm: GridMap,
        start: tuple[int, int],
        goal: tuple[int, int],
        diag: bool,
        weight: float,
    ) -> PlanResult:
        cfg = self.config
        h_name = cfg.heuristic
        neighbors = NEIGHBORS_8 if diag else NEIGHBORS_4
        h, w = gm.shape

        g_score: dict[tuple[int, int], float] = {start: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        closed: set[tuple[int, int]] = set()

        def hval(node: tuple[int, int]) -> float:
            return weight * heuristic(goal[0] - node[0], goal[1] - node[1], h_name, diag)

        counter = 0  # 单调计数器：堆元素不可比较时做稳定排序
        heap: list[tuple[float, float, int, tuple[int, int]]] = []
        heapq.heappush(heap, (hval(start), 0.0, counter, start))

        expansions = 0
        while heap:
            _f, _h, _c, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)
            expansions += 1

            if cur == goal:
                path = self._reconstruct(parent, start, goal)
                return PlanResult(
                    True, path, g_score[goal], expansions, "", diag
                )

            if expansions > cfg.max_expansions:
                return PlanResult(
                    False,
                    reason=f"超过扩展上限 {cfg.max_expansions}（地图过大或目标不可达）",
                    expansions=expansions,
                    allow_diagonal=diag,
                )

            ci, cj = cur
            g_cur = g_score[cur]
            for di, dj, step in neighbors:
                ni, nj = ci + di, cj + dj
                if not (0 <= ni < h and 0 <= nj < w):
                    continue
                if gm.grid[ni, nj] != 0:
                    continue
                # 对角穿角（严格版）：两侧**任一格**是障碍就禁止斜穿。
                # 只禁止"两侧都占用"虽然栅格层面合法，但连接两个格心的直线
                # 仍可能擦过占用格的一角，后处理用连续空间视线检查时会判定碰撞。
                # 这里提前收紧，保证 A* 输出与 smoothing 的碰撞语义一致。
                if di != 0 and dj != 0:
                    if gm.grid[ci, nj] != 0 or gm.grid[ni, cj] != 0:
                        continue
                nb = (ni, nj)
                if nb in closed:
                    continue
                tentative = g_cur + step
                if tentative + 1e-12 < g_score.get(nb, math.inf):
                    g_score[nb] = tentative
                    parent[nb] = cur
                    counter += 1
                    hv = hval(nb)
                    prio = tentative + hv
                    # 平局打破：优先 h 小（更接近目标）
                    heapq.heappush(
                        heap, (prio, hv if cfg.tie_break else 0.0, counter, nb)
                    )

        return PlanResult(False, reason="OPEN 表耗尽，目标不可达", expansions=expansions, allow_diagonal=diag)

    @staticmethod
    def _reconstruct(
        parent: dict[tuple[int, int], tuple[int, int]],
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [goal]
        cur = goal
        while cur != start:
            cur = parent[cur]
            path.append(cur)
        path.reverse()
        return path


# ---------------- 路径度量与后处理工具 ----------------


def path_length(cells: list[tuple[int, int]]) -> float:
    """栅格路径的几何长度（以格为单位）。"""
    if len(cells) < 2:
        return 0.0
    a = np.asarray(cells, dtype=float)
    d = np.diff(a, axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def path_world(gm: GridMap, cells: list[tuple[int, int]]) -> np.ndarray:
    """栅格路径 -> 世界坐标数组 ``(N, 2)``。"""
    if not cells:
        return np.zeros((0, 2))
    return np.asarray([gm.to_world(i, j) for i, j in cells], dtype=float)


def count_turns(cells: list[tuple[int, int]]) -> int:
    """统计路径的方向变化次数，衡量"锯齿程度"。"""
    if len(cells) < 3:
        return 0
    a = np.asarray(cells, dtype=float)
    d = np.diff(a, axis=0)
    ang = np.arctan2(d[:, 0], d[:, 1])
    turns = np.abs(np.diff(ang))
    turns = np.minimum(turns, 2 * math.pi - turns)
    return int(np.sum(turns > 1e-6))
