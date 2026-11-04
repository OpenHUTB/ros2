#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""固定航线几何：闭合航线定义、最近点投影、横向偏差与进度.

与 uav_ppo_nav 的区别
---------------------
uav_ppo_nav 是「随机起点 → 随机目标点，到达即结束」；
本模块是「沿一条**预先给定**的闭合航线巡航」—— 任务从"点对点"变成"循迹"，
所以观测里要给出**航线几何关系**（横向偏差、航向偏差、前方航点），
评价指标也要换成**路线完成度 / 平均横向偏差 / 完成时间**。

坐标系
------
全部世界系 ENU（东 x、北 y、天 z）。与既有模块保持一致，
避免机头偏航漂移影响控制（CarlaAir 的 SimpleFlight 偏航保持不生效）。
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

# ---- 默认航线：20 m × 14 m 的圆角矩形（闭合），高度 1.5 m ----
DEFAULT_HEIGHT = 1.5
CORNER_RADIUS = 3.0
CORNER_SEGMENTS = 6


def _arc(cx: float, cy: float, r: float, a0: float, a1: float,
         seg: int) -> List[Tuple[float, float]]:
    """圆弧采样（含端点），a0/a1 为弧度。"""
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / seg),
             cy + r * math.sin(a0 + (a1 - a0) * i / seg))
            for i in range(seg + 1)]


def default_route(width: float = 20.0, depth: float = 14.0,
                  radius: float = CORNER_RADIUS,
                  segments: int = CORNER_SEGMENTS) -> np.ndarray:
    """生成一条闭合圆角矩形航线，返回 (N, 3) 世界坐标（首尾相接，末点等于首点的下一段）. """
    r = min(radius, min(width, depth) / 2.0 - 0.1)
    hx, hy = width / 2.0 - r, depth / 2.0 - r
    pts: List[Tuple[float, float]] = []
    # 右上 → 右下 → 左下 → 左上 四个角依次倒角
    pts += _arc(hx, hy, r, 0.0, math.pi / 2, segments)              # 右上角
    pts += _arc(-hx, hy, r, math.pi / 2, math.pi, segments)         # 左上角
    pts += _arc(-hx, -hy, r, math.pi, 1.5 * math.pi, segments)      # 左下角
    pts += _arc(hx, -hy, r, 1.5 * math.pi, 2.0 * math.pi, segments)  # 右下角
    arr = np.array(pts, dtype=np.float64)
    arr = np.vstack([arr, arr[:1]])          # 闭合
    out = np.zeros((len(arr), 3), dtype=np.float64)
    out[:, :2] = arr
    out[:, 2] = DEFAULT_HEIGHT
    return out


class Route(object):
    """闭合航线：提供最近点投影、横向偏差、路线进度、前方航点。

    参数
    ----
    points : (N, 3) 世界坐标，按巡航方向排列，首尾自动闭合
    """

    def __init__(self, points: Sequence[Sequence[float]]):
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if len(pts) < 2:
            raise ValueError("航线至少需要 2 个点")
        if not np.allclose(pts[0], pts[-1]):
            pts = np.vstack([pts, pts[:1]])
        self.points = pts
        seg = pts[1:] - pts[:-1]
        self.seg = seg
        self.seg_len = np.linalg.norm(seg[:, :2], axis=1)
        self.seg_len[self.seg_len < 1e-9] = 1e-9
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])

    # -------------------------------------------------------------- 投影
    def project(self, pos_xy) -> Tuple[int, float, np.ndarray, float]:
        """把水平位置投影到航线上。

        返回 (分段索引, 弧长进度 s, 航线上最近点 (2,), 横向偏差 signed)。
        横向偏差的正负按「沿航线方向的左侧为正」定义。
        """
        p = np.asarray(pos_xy, dtype=np.float64).reshape(2)
        a = self.points[:-1, :2]
        d = self.seg[:, :2]
        t = np.clip(((p - a) * d).sum(axis=1) / (self.seg_len ** 2), 0.0, 1.0)
        proj = a + d * t[:, None]
        dist = np.linalg.norm(p - proj, axis=1)
        i = int(np.argmin(dist))
        s = float(self.cum[i] + t[i] * self.seg_len[i])
        # 左侧为正：叉乘的 z 分量
        cross = d[i, 0] * (p[1] - a[i, 1]) - d[i, 1] * (p[0] - a[i, 0])
        signed = float(cross / self.seg_len[i])
        return i, s, proj[i], signed

    def waypoint_ahead(self, pos_xy, s: float, lookahead: float = 4.0) -> np.ndarray:
        """取航线上「进度 s + lookahead」处的点（环绕），返回 (2,). """
        target = (s + lookahead) % self.length
        # 二分找分段
        idx = int(np.searchsorted(self.cum, target, side="right") - 1)
        idx = min(max(idx, 0), len(self.seg_len) - 1)
        frac = (target - self.cum[idx]) / self.seg_len[idx]
        p = self.points[idx, :2] + self.seg[idx, :2] * frac
        return p

    def progress_ratio(self, s: float) -> float:
        """弧长进度 -> [0, 1] 比例。"""
        return float((s % self.length) / self.length)

    # -------------------------------------------------------------- 指标
    def completion(self, visited_s: Sequence[float], bins: int = 36) -> float:
        """路线完成度：按弧长把航线分成 bins 段，统计被访问过的段占比。"""
        if len(visited_s) == 0:
            return 0.0
        idx = (np.asarray(visited_s, dtype=np.float64) % self.length) / self.length
        hit = np.zeros(bins, dtype=bool)
        hit[np.clip((idx * bins).astype(np.int64), 0, bins - 1)] = True
        return float(hit.mean())


def make_route(name: str = "rect") -> Route:
    """按名字取一条内置航线。"""
    name = (name or "rect").lower()
    if name in ("rect", "rectangle", "box"):
        return Route(default_route())
    if name in ("figure8", "eight"):
        # 八字形：两个相切的圆
        pts = []
        for sign in (1, -1):
            pts += _arc(0.0, 0.0, 6.0, 0.0, math.pi, 8)[::sign]
        arr = np.array(pts + pts[:1])
        out = np.zeros((len(arr), 3))
        out[:, 0] = arr[:, 0]
        out[:, 1] = arr[:, 1]
        out[:, 2] = DEFAULT_HEIGHT
        return Route(out)
    raise ValueError("未知航线：%s（可选 rect / figure8）" % name)
