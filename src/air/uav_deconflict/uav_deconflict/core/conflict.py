#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""时空冲突检测：解析求解两条 4D 航迹的最近接近（CPA）。

低空空域把安全间隔定义成一个**圆柱**：水平间隔 ``sep_h`` 与垂直间隔
``sep_v`` 必须同时满足。因此判定"是否冲突"不能只看三维距离，而要解出

    |Δp_xy(t)| < sep_h   且   |Δp_z(t)| < sep_v

同时成立的时间区间。两条航迹都是分段匀速直线，在每一对时间重叠的航段内
相对运动是匀速直线，上述两个条件各自化为二次/一次不等式，可以直接解出
闭区间，再求交。整个过程没有采样步长，结论与实现无关。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .plan import Plan, Segment


@dataclass(frozen=True)
class Separation:
    """安全间隔（圆柱模型）。"""

    horizontal: float = 50.0     # m
    vertical: float = 20.0       # m

    def __post_init__(self) -> None:
        if self.horizontal <= 0.0 or self.vertical <= 0.0:
            raise ValueError("间隔必须为正: %r" % (self,))


@dataclass
class Conflict:
    """一段持续存在的间隔侵入。"""

    a: str
    b: str
    t_start: float
    t_end: float
    min_h: float
    min_v: float
    cpa_t: float
    sep: Separation

    @property
    def duration(self) -> float:
        return max(0.0, self.t_end - self.t_start)

    @property
    def pair(self) -> Tuple[str, str]:
        return (self.a, self.b)

    def severity(self) -> float:
        """0 = 刚好触碰间隔边界，1 = 完全重合（用于排序与着色）。"""
        rh = 1.0 - min(1.0, self.min_h / self.sep.horizontal)
        rv = 1.0 - min(1.0, self.min_v / self.sep.vertical)
        return max(rh, rv)

    def describe(self) -> str:
        return ("%s↔%s  t=%.1f~%.1f s  最小水平 %.1f m / 垂直 %.1f m"
                % (self.a, self.b, self.t_start, self.t_end,
                   self.min_h, self.min_v))


# --------------------------------------------------------------------------- #
# 区间求解
# --------------------------------------------------------------------------- #
def _interval_in_circle(d0: np.ndarray, v: np.ndarray, radius: float,
                        tau_lo: float, tau_hi: float) -> Optional[Tuple[float, float]]:
    """求 |d0 + v·τ| ≤ radius 的 τ 区间（τ 以段重合起点为 0，区间为 [tau_lo, tau_hi]）。"""
    a = float(np.dot(v, v))
    b = 2.0 * float(np.dot(d0, v))
    c = float(np.dot(d0, d0)) - radius * radius
    if a <= 1e-12:
        # 相对静止：恰好等于间隔标准按"相切"处理，不算侵入（严格小于）
        return (tau_lo, tau_hi) if c < 0.0 else None
    disc = b * b - 4.0 * a * c
    if disc <= 0.0:
        return None
    root = math.sqrt(disc)
    t1 = (-b - root) / (2.0 * a)
    t2 = (-b + root) / (2.0 * a)
    left, right = max(tau_lo, min(t1, t2)), min(tau_hi, max(t1, t2))
    return (left, right) if right >= left else None


def _interval_in_slab(z0: float, vz: float, half: float,
                      tau_lo: float, tau_hi: float) -> Optional[Tuple[float, float]]:
    """求 |z0 + vz·τ| ≤ half 的 τ 区间。"""
    if abs(vz) <= 1e-12:
        return (tau_lo, tau_hi) if abs(z0) <= half else None
    t1 = (-half - z0) / vz
    t2 = (half - z0) / vz
    left, right = max(tau_lo, min(t1, t2)), min(tau_hi, max(t1, t2))
    return (left, right) if right >= left else None


def _min_norm_in_interval(d0: np.ndarray, v: np.ndarray,
                          lo: float, hi: float) -> Tuple[float, float]:
    """|d0 + v·τ| 在 [lo, hi] 上的最小值及其时刻（相对 τ）。"""
    a = float(np.dot(v, v))
    if a <= 1e-12:
        return float(np.linalg.norm(d0)), lo
    tau = -float(np.dot(d0, v)) / a
    tau = min(max(tau, lo), hi)
    return float(np.linalg.norm(d0 + v * tau)), tau


def _merge(intervals: List[Tuple[float, float]],
           gap: float) -> List[Tuple[float, float]]:
    if not intervals:
        return []
    intervals = sorted(intervals)
    out = [list(intervals[0])]
    for lo, hi in intervals[1:]:
        if lo <= out[-1][1] + gap:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return [(a, b) for a, b in out]


# --------------------------------------------------------------------------- #
# 两机检测
# --------------------------------------------------------------------------- #
def pair_conflicts(a: Plan, b: Plan, sep: Separation,
                   merge_gap: float = 0.5) -> List[Conflict]:
    """返回 A、B 之间所有间隔侵入时段（已合并相邻时段）。"""
    raw: List[Tuple[float, float, float, float, float]] = []

    for sa in a.segments:
        for sb in b.segments:
            lo = max(sa.t0, sb.t0)
            hi = min(sa.t1, sb.t1)
            if hi - lo <= 1e-9:
                continue
            pa = sa.position(lo)
            pb = sb.position(lo)
            v = sa.velocity - sb.velocity
            d0 = pa - pb
            span = hi - lo

            h_iv = _interval_in_circle(d0[:2], v[:2], sep.horizontal, 0.0, span)
            if h_iv is None:
                continue
            v_iv = _interval_in_slab(d0[2], v[2], sep.vertical, 0.0, span)
            if v_iv is None:
                continue
            left = max(h_iv[0], v_iv[0])
            right = min(h_iv[1], v_iv[1])
            if right < left:
                continue

            # 在该时段内分别求水平与垂直的最小值；τ 相对段重合起点，
            # 入库前统一换成绝对时间。
            mh, tau_h = _min_norm_in_interval(d0[:2], v[:2], left, right)
            mv, _ = _min_norm_in_interval(d0[2:3], v[2:3], left, right)
            raw.append((lo + left, lo + right, mh, mv, lo + tau_h))

    merged = _merge([(lo, hi) for lo, hi, _, _, _ in raw], merge_gap)
    out: List[Conflict] = []
    for lo, hi in merged:
        inside = [r for r in raw if r[0] >= lo - 1e-9 and r[1] <= hi + 1e-9]
        if not inside:
            continue
        min_h = min(r[2] for r in inside)
        min_v = min(r[3] for r in inside)
        # CPA 取水平最接近的时刻
        cpa = min(inside, key=lambda r: r[2])[4]
        out.append(Conflict(a.uid, b.uid, lo, hi, min_h, min_v, cpa, sep))
    return out


def detect(plans: Sequence[Plan], sep: Separation,
           merge_gap: float = 0.5) -> List[Conflict]:
    """全机队两两检测，按开始时刻排序。"""
    out: List[Conflict] = []
    items = list(plans)
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            out.extend(pair_conflicts(items[i], items[j], sep, merge_gap))
    out.sort(key=lambda c: (c.t_start, c.a, c.b))
    return out


def conflict_matrix(plans: Sequence[Plan], sep: Separation) -> Dict[Tuple[str, str], int]:
    """机对 → 冲突段数，用于报告与热力图。"""
    mat: Dict[Tuple[str, str], int] = {}
    for c in detect(plans, sep):
        mat[c.pair] = mat.get(c.pair, 0) + 1
    return mat
