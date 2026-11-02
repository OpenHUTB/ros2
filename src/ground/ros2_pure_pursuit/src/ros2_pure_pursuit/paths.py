"""参考路径的生成与查询工具。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Path:
    """离散参考路径，形状为 (N, 2)。"""

    points: np.ndarray
    closed: bool = False   # 闭合路径（如圆、8 字）不做终点判定，改为圈数判定

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=float).reshape(-1, 2)

    def __len__(self) -> int:
        return len(self.points)

    @property
    def length(self) -> float:
        d = np.diff(self.points, axis=0)
        return float(np.sum(np.hypot(d[:, 0], d[:, 1])))

    def nearest_index(self, x: float, y: float) -> int:
        """返回距离 (x, y) 最近的路径点索引。"""
        d = np.hypot(self.points[:, 0] - x, self.points[:, 1] - y)
        return int(np.argmin(d))

    def lookahead_point(self, index: int, ld: float) -> tuple[np.ndarray, bool]:
        """从 index 起沿路径前进，找到第一个超出前视距离的点。

        采用线性插值保证前视点在路径上连续，返回 (点, 是否到达终点)。
        """
        pts = self.points
        acc = 0.0
        for i in range(index, len(pts) - 1):
            seg = np.linalg.norm(pts[i + 1] - pts[i])
            if acc + seg >= ld:
                t = (ld - acc) / seg if seg > 1e-9 else 0.0
                return pts[i] + t * (pts[i + 1] - pts[i]), False
            acc += seg
        return pts[-1], True

    def cross_track_error(self, index: int, x: float, y: float) -> float:
        """到最近路径段的横向跟踪误差（带符号由叉积决定）。"""
        p0 = self.points[index]
        p1 = self.points[min(index + 1, len(self.points) - 1)]
        seg = p1 - p0
        norm = np.linalg.norm(seg)
        if norm < 1e-9:
            return float(np.hypot(x - p0[0], y - p0[1]))
        cross = seg[0] * (y - p0[1]) - seg[1] * (x - p0[0])
        return float(cross / norm)


def circle(radius: float = 8.0, n: int = 400) -> Path:
    """生成闭合圆形路径。"""
    t = np.linspace(0.0, 2.0 * np.pi, n)
    return Path(np.stack([radius * np.cos(t), radius * np.sin(t)], axis=1), closed=True)


def figure_eight(a: float = 10.0, n: int = 400) -> Path:
    """生成 8 字形路径（利萨如曲线）。"""
    t = np.linspace(0.0, 2.0 * np.pi, n)
    return Path(np.stack([a * np.sin(t), a * np.sin(t) * np.cos(t)], axis=1), closed=True)


def straight(length: float = 20.0, n: int = 200) -> Path:
    """生成沿 x 轴的直线路径。"""
    return Path(np.stack([np.linspace(0.0, length, n), np.zeros(n)], axis=1))


def s_curve(length: float = 24.0, amp: float = 3.0, n: int = 300) -> Path:
    """生成 S 形路径，用于检验曲率突变处的跟踪性能。"""
    x = np.linspace(0.0, length, n)
    y = amp * np.sin(2.0 * np.pi * x / length)
    return Path(np.stack([x, y], axis=1))


def load_path(path_file: str) -> Path:
    """从 csv 读取参考路径，兼容带表头与不带表头两种格式。"""
    try:
        data = np.loadtxt(path_file, delimiter=",", comments="#")
    except ValueError:
        data = np.loadtxt(path_file, delimiter=",", skiprows=1, comments="#")
    return Path(np.atleast_2d(data))
