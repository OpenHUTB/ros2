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


    def curvature(self, index: int, span: float = 0.8) -> float:
        """用 index 附近弧长 span 的三点估计路径曲率（1/m，带符号）。

        三点外接圆曲率 ``kappa = 2 * A / (a * b * c)``，其中 A 是三角形带符号面积。
        取"跨一段弧长"而不是相邻两点，是为了压掉离散化的锯齿噪声；
        闭合路径用取模回绕，开放路径在端点处退化为可用点。
        """
        pts = self.points
        n = len(pts)
        if n < 3:
            return 0.0

        def walk(start: int, step: int, target: float) -> int:
            """沿路径走，直到"弧长够"且"至少跨 2 个点"（稀疏路点下也够用）."""
            acc, i, hops = 0.0, start, 0
            while acc < target or hops < 2:
                j = i + step
                if self.closed:
                    j %= n
                elif j < 0 or j >= n:
                    break
                acc += float(np.linalg.norm(pts[j] - pts[i]))
                i = j
                hops += 1
            return i

        i0 = walk(index, -1, span * 0.5)
        i2 = walk(index, +1, span * 0.5)
        p0, p1, p2 = pts[i0], pts[index], pts[i2]
        a = float(np.linalg.norm(p1 - p0))
        b = float(np.linalg.norm(p2 - p1))
        c = float(np.linalg.norm(p2 - p0))
        if a * b * c < 1e-12:
            return 0.0
        cross = float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]))
        return 2.0 * cross / (a * b * c)

    def resample(self, spacing: float) -> "Path":
        """按弧长等距重采样，得到"稀疏路点"版本（保持起终点与闭合性）。

        上游给的路径未必是稠密折线（有的只给每 1~2 m 一个路点），
        ``curvature()`` 依赖"跨弧长取点"，路点稀疏时窗口内可能只有两三个点；
        这里提供统一的稀疏化/加密工具，便于做稀疏路点的鲁棒性实验。
        """
        if spacing <= 1e-6:
            raise ValueError("spacing 必须为正数")
        pts = self.points
        if len(pts) < 2:
            return Path(pts.copy(), closed=self.closed)

        loop = np.vstack([pts, pts[0]]) if self.closed else pts
        seg = np.linalg.norm(np.diff(loop, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(s[-1])
        if total < 1e-9:
            return Path(pts.copy(), closed=self.closed)

        count = max(2, int(round(total / spacing)) + 1)
        targets = np.linspace(0.0, total, count)
        out = np.stack([np.interp(targets, s, loop[:, 0]),
                        np.interp(targets, s, loop[:, 1])], axis=1)
        if self.closed:
            out = out[:-1]          # 去掉与起点重合的闭合点
        return Path(out, closed=self.closed)


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
