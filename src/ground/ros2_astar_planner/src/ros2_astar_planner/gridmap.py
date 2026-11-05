"""占用栅格地图：坐标换算、障碍物膨胀、连通性检查与地图生成。

设计上刻意保持零 ROS 依赖，只用 NumPy，这样规划器可以在纯 Python 环境里
做单元测试和离线仿真；ROS2 节点只负责把 nav_msgs/OccupancyGrid 转成这里的
:class:`GridMap`。

约定
----
- 栅格数组 ``grid`` 形状为 ``(H, W)``，``grid[i, j]`` 中 ``i`` 是行（y 方向）、
  ``j`` 是列（x 方向）；取值 0 表示自由，1 表示占用。
- 世界坐标原点在栅格 ``(0, 0)`` 的**左下角**，与 ROS OccupancyGrid 一致。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    """栅格地图的几何参数。"""

    width: int      # 列数（x 方向格数）
    height: int     # 行数（y 方向格数）
    resolution: float  # 每格边长 (m)
    origin_x: float = 0.0  # 栅格 (0,0) 左下角的世界 x
    origin_y: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("栅格尺寸必须为正")
        if self.resolution <= 0.0:
            raise ValueError("分辨率必须为正")


class GridMap:
    """占用栅格地图。

    Args:
        grid: ``(H, W)`` 数组，非零即占用；会自动转成 int8 的 0/1。
        spec: 几何参数。
    """

    def __init__(self, grid: np.ndarray, spec: GridSpec) -> None:
        g = np.asarray(grid)
        if g.ndim != 2:
            raise ValueError("grid 必须是二维数组")
        if g.shape != (spec.height, spec.width):
            raise ValueError(
                f"grid 形状 {g.shape} 与 spec 不匹配 "
                f"({spec.height}, {spec.width})"
            )
        self.spec = spec
        self.grid = (g != 0).astype(np.int8)

    # ---------------- 构造与基本属性 ----------------

    @classmethod
    def empty(cls, width: int, height: int, resolution: float = 0.1) -> "GridMap":
        """生成一张全空的地图。"""
        spec = GridSpec(width, height, resolution)
        return cls(np.zeros((height, width), dtype=np.int8), spec)

    @property
    def shape(self) -> tuple[int, int]:
        return self.grid.shape

    def copy(self) -> "GridMap":
        return GridMap(self.grid.copy(), self.spec)

    # ---------------- 坐标换算 ----------------

    def to_grid(self, x: float, y: float) -> tuple[int, int]:
        """世界坐标 -> 栅格索引 ``(i, j)``；不做边界裁剪。"""
        s = self.spec
        j = int(np.floor((x - s.origin_x) / s.resolution))
        i = int(np.floor((y - s.origin_y) / s.resolution))
        return i, j

    def to_world(self, i: int, j: int) -> tuple[float, float]:
        """栅格索引 -> 格心世界坐标。"""
        s = self.spec
        x = s.origin_x + (j + 0.5) * s.resolution
        y = s.origin_y + (i + 0.5) * s.resolution
        return x, y

    # ---------------- 查询 ----------------

    def in_bounds(self, i: int, j: int) -> bool:
        h, w = self.grid.shape
        return 0 <= i < h and 0 <= j < w

    def is_free(self, i: int, j: int) -> bool:
        """格心是否可通行；越界一律视为不可通行。"""
        if not self.in_bounds(i, j):
            return False
        return bool(self.grid[i, j] == 0)

    def free_cells(self) -> np.ndarray:
        """返回所有自由格的 ``(i, j)`` 数组，形状 ``(N, 2)``。"""
        ii, jj = np.nonzero(self.grid == 0)
        return np.stack([ii, jj], axis=1)

    # ---------------- 编辑 ----------------

    def set_obstacle(self, i: int, j: int) -> None:
        if self.in_bounds(i, j):
            self.grid[i, j] = 1

    def add_rect(self, i0: int, j0: int, i1: int, j1: int) -> None:
        """填充一个闭区间矩形障碍。"""
        h, w = self.grid.shape
        i0, i1 = max(0, i0), min(h - 1, i1)
        j0, j1 = max(0, j0), min(w - 1, j1)
        if i0 <= i1 and j0 <= j1:
            self.grid[i0 : i1 + 1, j0 : j1 + 1] = 1

    def inflate(self, radius_cells: int) -> "GridMap":
        """按切比雪夫距离膨胀障碍（保留机器人半径的安全间隙）。

        用 separable min-filter 实现：先横向、再纵向取邻域最小值，
        等价于方形结构元的膨胀，复杂度 O(HW) 且与半径无关。
        """
        r = int(radius_cells)
        if r <= 0:
            return self.copy()
        g = self.grid.astype(np.int8)
        # 横向
        padded = np.pad(g, ((0, 0), (r, r)), mode="constant", constant_values=1)
        acc = padded.copy()
        for k in range(1, r + 1):
            acc[:, r:-r] |= padded[:, r - k : padded.shape[1] - r - k]
        # 纵向
        padded2 = np.pad(acc, ((r, r), (0, 0)), mode="constant", constant_values=1)
        acc2 = padded2.copy()
        for k in range(1, r + 1):
            acc2[r:-r, :] |= padded2[r - k : padded2.shape[0] - r - k, :]
        # 去掉两侧 padding，恢复 (H, W)
        return GridMap(acc2[r:-r, r:-r], self.spec)

    def distance_to_obstacle(self) -> np.ndarray:
        """每个自由格到最近障碍的**切比雪夫步数**（8-邻接多源 BFS）。

        切比雪夫距离与膨胀使用的方形结构元一致：距离 1 表示"有一个障碍格
        与之相邻"，距离 2 表示膨胀 2 格后该格会被覆盖。用途是挑选
        "离墙足够远"的起终点，避免机器人半径膨胀层把端点吞掉。
        """
        h, w = self.grid.shape
        dist = np.full((h, w), -1, dtype=np.int32)
        queue: list[tuple[int, int]] = []
        for i in range(h):
            for j in range(w):
                if self.grid[i, j] != 0:
                    dist[i, j] = 0
                    queue.append((i, j))
        if not queue:
            return dist
        head = 0
        while head < len(queue):
            i, j = queue[head]
            head += 1
            d = dist[i, j]
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = i + di, j + dj
                    if not (0 <= ni < h and 0 <= nj < w):
                        continue
                    if dist[ni, nj] >= 0:
                        continue
                    dist[ni, nj] = d + 1
                    queue.append((ni, nj))
        return dist

    # ---------------- 连通性 ----------------

    def connected_components(self) -> tuple[np.ndarray, int]:
        """对自由格做 4-邻接连通域标记。

        Returns:
            ``(labels, n)``，``labels`` 与 grid 同形，占用格为 -1，
            自由格为所属连通域编号 ``0..n-1``。
        """
        h, w = self.grid.shape
        labels = np.full((h, w), -1, dtype=np.int32)
        n = 0
        stack: list[tuple[int, int]] = []
        for i in range(h):
            for j in range(w):
                if self.grid[i, j] != 0 or labels[i, j] >= 0:
                    continue
                labels[i, j] = n
                stack.append((i, j))
                while stack:
                    ci, cj = stack.pop()
                    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        ni, nj = ci + di, cj + dj
                        if (
                            0 <= ni < h
                            and 0 <= nj < w
                            and self.grid[ni, nj] == 0
                            and labels[ni, nj] < 0
                        ):
                            labels[ni, nj] = n
                            stack.append((ni, nj))
                n += 1
        return labels, n

    def same_component(self, a: tuple[int, int], b: tuple[int, int]) -> bool:
        """两点之间是否存在 4-邻接通路（用于提前判定不可达）。"""
        if not (self.is_free(*a) and self.is_free(*b)):
            return False
        labels, _ = self.connected_components()
        return bool(labels[a[0], a[1]] == labels[b[0], b[1]])


# ---------------- 地图生成 ----------------


def random_obstacle_map(
    width: int,
    height: int,
    resolution: float,
    obstacle_ratio: float,
    seed: int,
    border: bool = True,
) -> GridMap:
    """生成随机障碍地图（伯努利撒点）。

    Args:
        obstacle_ratio: 障碍占比 0~1。
        border: 是否围一圈边界墙。
    """
    if not 0.0 <= obstacle_ratio <= 1.0:
        raise ValueError("obstacle_ratio 必须在 [0, 1]")
    rng = np.random.default_rng(seed)
    g = (rng.random((height, width)) < obstacle_ratio).astype(np.int8)
    if border:
        g[0, :] = 1
        g[-1, :] = 1
        g[:, 0] = 1
        g[:, -1] = 1
    return GridMap(g, GridSpec(width, height, resolution))


def corridor_map(
    width: int = 61,
    height: int = 41,
    resolution: float = 0.1,
    gap_rows: int = 9,
) -> GridMap:
    """走廊 + 房间场景：模拟室内结构化环境。

    Args:
        gap_rows: 每道墙上门洞的高度（格数）。门洞越窄，机器人越难通过，
            可用于研究"通道宽度 vs 机器人半径"的通行可行性。
    """
    m = GridMap.empty(width, height, resolution)
    # 外墙
    m.add_rect(0, 0, height - 1, width - 1)
    inner_i0, inner_i1 = 1, height - 2
    inner_j0, inner_j1 = 1, width - 2
    m.grid[inner_i0 : inner_i1 + 1, inner_j0 : inner_j1 + 1] = 0
    # 三道竖墙，各留一个门洞
    gap = max(1, int(gap_rows))
    for jcol, gap_i in ((15, 8), (30, height - 12), (45, 8)):
        m.add_rect(inner_i0, jcol, inner_i1, jcol)
        lo = max(inner_i0, gap_i)
        hi = min(inner_i1, gap_i + gap - 1)
        m.grid[lo : hi + 1, jcol] = 0
    # 房间内若干矩形障碍
    m.add_rect(6, 5, 12, 10)
    m.add_rect(25, 20, 33, 26)
    m.add_rect(6, 36, 14, 42)
    m.add_rect(28, 50, 34, 56)
    return m


def gate_map(
    gap_rows: int = 5,
    room: int = 7,
    resolution: float = 0.1,
) -> GridMap:
    """"一墙一门"地图：左右两个房间，中间一堵墙，墙上开一个门洞。

    专门用于研究**通道宽度与机器人半径的关系**：

    - 房间大小固定为 ``room × room`` 格，起点/终点位于房间中央，
      距墙足够远，因此连通性**只由门洞宽度决定**，不受端点位置干扰；
    - 门洞高度为 ``gap_rows`` 格、垂直居中。

    Args:
        gap_rows: 门洞高度（格）。
        room: 单个房间的边长（格）。
    """
    gap = max(1, int(gap_rows))
    r = max(3, int(room))
    width = 2 * r + 3
    height = 2 * r + 3
    m = GridMap.empty(width, height, resolution)
    # 外墙
    m.add_rect(0, 0, height - 1, width - 1)
    m.grid[1 : height - 1, 1 : width - 1] = 0
    # 中间墙
    mid_j = r + 1
    m.add_rect(1, mid_j, height - 2, mid_j)
    # 居中的门洞
    lo = max(1, (height - gap) // 2)
    hi = min(height - 2, lo + gap - 1)
    m.grid[lo : hi + 1, mid_j] = 0
    return m


def maze_map(width: int = 61, height: int = 41, resolution: float = 0.1) -> GridMap:
    """规则迷宫：大量死路，用于考察启发式与搜索效率。"""
    m = GridMap.empty(width, height, resolution)
    m.add_rect(0, 0, height - 1, width - 1)
    m.grid[1 : height - 1, 1 : width - 1] = 0
    # 竖墙：偶数列起，交替从上下伸出，留出蛇形通道
    for jcol in range(4, width - 3, 4):
        top = ((jcol // 4) % 2) == 0
        if top:
            m.add_rect(1, jcol, height - 8, jcol)
        else:
            m.add_rect(7, jcol, height - 2, jcol)
    return m
