"""占据栅格地图：固定场景、世界坐标与栅格坐标互转。"""
from __future__ import annotations

import numpy as np

RES = 0.1          # 米/格
W = 120            # 列数（x 方向）
H = 120            # 行数（y 方向）

# 障碍方块，单位米：(x0, y0, x1, y1)
BOXES = (
    (1.2, 1.2, 3.2, 2.4),
    (4.5, 0.8, 5.4, 6.0),
    (7.2, 2.0, 10.4, 3.0),
    (2.0, 4.6, 6.2, 5.4),
    (7.4, 5.6, 8.4, 10.2),
    (3.0, 7.8, 6.6, 8.8),
    (9.2, 7.0, 10.8, 9.0),
)

__all__ = ["RES", "W", "H", "INFLATE_CELLS", "build_grid", "inflate",
           "planning_grid", "to_cell", "is_free", "world_to_cell",
           "cell_center", "sample_free_pose"]

#: 规划与观测使用膨胀 4 格（0.4 m）的地图，给 0.18 m 机体半径留余量
INFLATE_CELLS = 6


def build_grid() -> np.ndarray:
    """返回 (H, W) 的 uint8 栅格，1 表示占据。场景固定，保证所有人结果一致。"""
    g = np.zeros((H, W), np.uint8)
    g[0:2, :] = 1
    g[H - 2:H, :] = 1
    g[:, 0:2] = 1
    g[:, W - 2:W] = 1
    for x0, y0, x1, y1 in BOXES:
        i0, i1 = int(round(x0 / RES)), int(round(x1 / RES))
        j0, j1 = int(round(y0 / RES)), int(round(y1 / RES))
        g[j0:j1, i0:i1] = 1
    return g


def inflate(g: np.ndarray, cells: int = 2) -> np.ndarray:
    """八邻域膨胀若干格，给规划器和观测留出机体半径的余量。"""
    out = g.copy()
    h, w = out.shape
    for _ in range(max(0, int(cells))):
        p = np.pad(out, 1, mode="constant", constant_values=1)
        acc = out.copy()
        for di in range(3):
            for dj in range(3):
                acc = np.maximum(acc, p[di:di + h, dj:dj + w])
        out = acc
    return out


def planning_grid() -> np.ndarray:
    """规划与观测用的膨胀地图（原始地图用于碰撞检测）。"""
    return inflate(build_grid(), INFLATE_CELLS)


def to_cell(g: np.ndarray, x: float, y: float):
    """世界坐标 → 栅格索引，浮点边界加小量修正（分辨率可能是 float32）。"""
    eps = 1e-3
    i = int(np.floor(x / RES + eps))
    j = int(np.floor(y / RES + eps))
    i = max(0, min(W - 1, i))
    j = max(0, min(H - 1, j))
    return i, j


def is_free(g: np.ndarray, x: float, y: float) -> bool:
    i, j = to_cell(g, x, y)
    return g[j, i] == 0


def world_to_cell(x: float, y: float):
    """不裁剪的整数栅格坐标，越界由调用方处理。"""
    return int(np.floor(x / RES + 1e-3)), int(np.floor(y / RES + 1e-3))


def cell_center(i: int, j: int):
    return (i + 0.5) * RES, (j + 0.5) * RES


def sample_free_pose(g: np.ndarray, rng: np.random.Generator,
                     margin: float = 0.6):
    """在空闲区域采样一个位姿 (x, y, yaw)。"""
    while True:
        x = float(rng.uniform(margin, W * RES - margin))
        y = float(rng.uniform(margin, H * RES - margin))
        if is_free(g, x, y):
            return x, y, float(rng.uniform(-np.pi, np.pi))
