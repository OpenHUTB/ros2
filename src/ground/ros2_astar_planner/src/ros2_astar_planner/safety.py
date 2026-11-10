"""安全与鲁棒性辅助：参数校验、端点修复、资源上限。

真实机器人上，路径规划最常见的故障不是"算法不收敛"，而是输入本身不合法：
目标点落在障碍里、地图全空/全占、参数被写成 0 或负数、传感器把地图发成了
半张……这些情况若不加处理，轻则规划出穿墙路径，重则节点卡死。本模块把
这些防御集中在一处，且不依赖 ROS，可被单元测试直接覆盖。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .gridmap import GridMap


def sanitize_positive(
    value: float,
    default: float,
    name: str,
    lo: float = 1e-6,
    hi: float = 1e6,
) -> tuple[float, str | None]:
    """把参数修正到合法区间，非法时回退默认值并给出告警文本。"""
    warning = None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default, f"{name} 不是数值，回退默认值 {default}"
    if not math.isfinite(v) or v <= 0.0:
        return default, f"{name}={value} 非法（非有限或非正），回退默认值 {default}"
    if v < lo:
        return lo, f"{name}={v} 过小，夹紧到 {lo}"
    if v > hi:
        return hi, f"{name}={v} 过大，夹紧到 {hi}"
    return v, warning


@dataclass
class EndpointFix:
    """端点修复结果。"""

    cell: tuple[int, int]
    changed: bool
    reason: str


def nearest_free_cell(gm: GridMap, cell: tuple[int, int], max_ring: int = 30) -> tuple[int, int] | None:
    """以 BFS 环形扩张寻找最近自由格；找不到返回 ``None``。

    ``max_ring`` 限制搜索半径，避免在全占用地图上无限扩张。
    """
    if gm.is_free(*cell):
        return cell
    h, w = gm.shape
    i0, j0 = cell
    queue = deque([(i0, j0)])
    seen = {(i0, j0)}
    while queue:
        i, j = queue.popleft()
        if abs(i - i0) > max_ring or abs(j - j0) > max_ring:
            continue
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                ni, nj = i + di, j + dj
                if (ni, nj) in seen or not (0 <= ni < h and 0 <= nj < w):
                    continue
                seen.add((ni, nj))
                if gm.is_free(ni, nj):
                    return (ni, nj)
                queue.append((ni, nj))
    return None


def resolve_endpoint(gm: GridMap, cell: tuple[int, int], name: str) -> EndpointFix:
    """把越界或落在障碍内的端点修复到最近自由格。"""
    if not gm.in_bounds(*cell):
        h, w = gm.shape
        clipped = (min(max(cell[0], 0), h - 1), min(max(cell[1], 0), w - 1))
        fixed = nearest_free_cell(gm, clipped)
        if fixed is None:
            return EndpointFix(clipped, True, f"{name} 越界且附近无可通行格")
        return EndpointFix(fixed, True, f"{name} 越界，已裁剪并移到最近自由格 {fixed}")
    if not gm.is_free(*cell):
        fixed = nearest_free_cell(gm, cell)
        if fixed is None:
            return EndpointFix(cell, False, f"{name} 位于障碍内且附近无可通行格")
        return EndpointFix(fixed, True, f"{name} 位于障碍内，已移到最近自由格 {fixed}")
    return EndpointFix(cell, False, "")


def choose_connected_endpoints(
    gm: GridMap,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> tuple[tuple[int, int], tuple[int, int], list[str]]:
    """返回一对保证连通的起终点，并记录所有自动修复说明。

    真实场景中用户点选的目标常常落在障碍内、或与被困在另一个房间的起点
    不连通。这里先做端点修复，再检查连通性；若仍不连通，则从**最大连通域**
    中确定性地挑出相距最远的两点，保证规划一定能给出结果，同时把调整
    写进日志让上层知道"这不是用户原本要的目标"。

    Returns:
        ``(start, goal, notes)``
    """
    notes: list[str] = []
    s = resolve_endpoint(gm, start, "起点")
    g = resolve_endpoint(gm, goal, "终点")
    for fix in (s, g):
        if fix.reason:
            notes.append(fix.reason)
    if gm.same_component(s.cell, g.cell):
        return s.cell, g.cell, notes

    labels, n = gm.connected_components()
    if n == 0:
        return s.cell, g.cell, notes + ["地图中没有自由格，无法规划"]
    sizes = [int(np.count_nonzero(labels == k)) for k in range(n)]
    k = int(np.argmax(sizes))
    cells = np.argwhere(labels == k)  # (N, 2)，每行 (i, j)

    # 四个投影极值点作为候选，两两取欧氏距离最大的一对（确定性、O(N)）
    proj = np.stack(
        [
            cells[:, 0] + cells[:, 1],
            cells[:, 0] - cells[:, 1],
            -cells[:, 0] + cells[:, 1],
            -cells[:, 0] - cells[:, 1],
        ],
        axis=1,
    )
    cand = set()
    for c in range(proj.shape[1]):
        cand.add(int(np.argmin(proj[:, c])))
        cand.add(int(np.argmax(proj[:, c])))
    cand = sorted(cand)
    best, best_d = None, -1.0
    for a in range(len(cand)):
        for b in range(a + 1, len(cand)):
            d = float(np.linalg.norm(cells[cand[a]] - cells[cand[b]]))
            if d > best_d:
                best_d, best = d, (cand[a], cand[b])
    assert best is not None
    new_s = (int(cells[best[0], 0]), int(cells[best[0], 1]))
    new_g = (int(cells[best[1], 0]), int(cells[best[1], 1]))
    notes.append(
        f"原起终点分属不同自由区域，已自动改为最大连通域（{sizes[k]} 格）内"
        f"相距最远的两点 {new_s} → {new_g}"
    )
    return new_s, new_g, notes


def pick_sturdy_endpoints(
    gm: GridMap,
    min_clearance: int = 2,
) -> tuple[tuple[int, int], tuple[int, int]]:
    """在最大连通域内挑一对"离墙足够远"的起终点（偏左下 / 偏右上）。

    与 :func:`choose_connected_endpoints` 的区别：这里**不修复**端点，
    而是从一开始就选取远离障碍的格子，使同一对端点在不同的膨胀半径下
    保持有效，便于做"膨胀半径 vs 路径长度"的公平对照实验。
    """
    labels, n = gm.connected_components()
    if n == 0:
        raise ValueError("地图中没有自由格")
    sizes = [int(np.count_nonzero(labels == k)) for k in range(n)]
    k = int(np.argmax(sizes))
    cells = np.argwhere(labels == k)
    dist = gm.distance_to_obstacle()

    ok = cells[dist[cells[:, 0], cells[:, 1]] >= min_clearance]
    if len(ok) == 0:
        ok = cells
    # 在同一连通域内，用四个投影极值点作为候选，两两取欧氏距离最大的一对。
    # 只用 i+j 的极值是不够的：走廊地图里左下角与右上角可能分属不同房间。
    proj = np.stack(
        [
            ok[:, 0] + ok[:, 1],
            ok[:, 0] - ok[:, 1],
            -ok[:, 0] + ok[:, 1],
            -ok[:, 0] - ok[:, 1],
        ],
        axis=1,
    )
    cand = sorted(
        {int(np.argmin(proj[:, c])) for c in range(proj.shape[1])}
        | {int(np.argmax(proj[:, c])) for c in range(proj.shape[1])}
    )
    best, best_d = None, -1.0
    for a in range(len(cand)):
        for b in range(a + 1, len(cand)):
            d = float(np.linalg.norm(ok[cand[a]] - ok[cand[b]]))
            if d > best_d:
                best_d, best = d, (cand[a], cand[b])
    assert best is not None
    start = ok[best[0]]
    goal = ok[best[1]]
    return (int(start[0]), int(start[1])), (int(goal[0]), int(goal[1]))


def validate_map(gm: GridMap) -> str | None:
    """地图整体合法性检查，返回问题描述或 ``None``。"""
    if gm.grid.size == 0:
        return "地图为空"
    if np.all(gm.grid != 0):
        return "地图全部被占用"
    if gm.spec.resolution <= 0:
        return "地图分辨率非正"
    return None


def inflation_cells(robot_radius: float, resolution: float) -> int:
    """按机器人半径计算需要膨胀的格数（向上取整，留出安全裕度）。"""
    r, warning = sanitize_positive(robot_radius, 0.15, "robot_radius")
    _ = warning
    res, _ = sanitize_positive(resolution, 0.1, "resolution")
    return max(0, int(math.ceil(r / res)))


class DeadlineGuard:
    """规划耗时上限保护；超时由调用方决定降级策略。"""

    def __init__(self, max_seconds: float = 2.0) -> None:
        self.max_seconds, _ = sanitize_positive(max_seconds, 2.0, "max_seconds")
        self._start: float | None = None

    def start(self) -> "DeadlineGuard":
        import time

        self._start = time.monotonic()
        return self

    def expired(self) -> bool:
        import time

        if self._start is None:
            return False
        return (time.monotonic() - self._start) > self.max_seconds
