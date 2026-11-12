#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drop point selection: greedy + swap improvement, multi-sortie packing.

Single-line summary: weight = urgency x demand greedy knapsack under cargo
capacity, 1-for-1 swap improvement, then nearest-neighbor route per sortie
with reload time between sorties.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DemandPoint:
    """需求点：坐标 + 需求份数 + 紧迫度（1~5）。"""

    pid: str
    x: float
    y: float
    demand: int
    urgency: int

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)

    @property
    def weight(self) -> float:
        return float(self.urgency * self.demand)


@dataclass(frozen=True)
class CargoCfg:
    """载货与架次配置。"""

    capacity: int = 12        # 单架次载货份数
    reload_s: float = 60.0    # 回补给点装载时间（s）
    cruise_speed: float = 8.0  # m/s
    depot: Tuple[float, float] = (0.0, 0.0)


@dataclass
class Mission:
    """一个架次的投送任务。"""

    sortie: int
    pids: List[str]
    load: int
    distance_m: float
    flight_s: float
    depart_s: float           # 含前一架次飞行 + 装载
    points: List[DemandPoint] = field(default_factory=list, repr=False)

    def as_dict(self) -> dict:
        return dict(sortie=self.sortie, pids=list(self.pids), load=self.load,
                    distance_m=round(self.distance_m, 1),
                    flight_s=round(self.flight_s, 1),
                    depart_s=round(self.depart_s, 1))


def greedy_select(points: Sequence[DemandPoint],
                  capacity: int) -> List[str]:
    """按 weight 降序装填，容量约束下第一步最大化加权覆盖。"""
    picked, load = [], 0
    for p in sorted(points, key=lambda q: (-q.weight, q.pid)):
        if load + p.demand <= capacity:
            picked.append(p.pid)
            load += p.demand
    return picked


def _covered_weight(points, pids) -> float:
    m = {p.pid: p for p in points}
    return sum(m[i].weight for i in pids)


def swap_improve(points: Sequence[DemandPoint], selected: List[str],
                 capacity: int) -> List[str]:
    """交换改进：移除 1 个已选点后按权重贪心回填若干未选点，加权覆盖
    只增不减，循环到无改进为止（1 换 1 是其特例）。"""
    cur = list(selected)
    cur_w = _covered_weight(points, cur)
    improved = True
    while improved:
        improved = False
        for q in sorted(cur, key=lambda k: next(
                p.weight for p in points if p.pid == k)):
            rest = [p for p in points if p.pid in cur and p.pid != q]
            load = sum(p.demand for p in rest)
            pool_ids = {p.pid for p in rest} | {q}
            adds, ld = [], load
            for p in sorted((q2 for q2 in points if q2.pid not in pool_ids),
                            key=lambda q2: (-q2.weight, q2.pid)):
                if ld + p.demand <= capacity:
                    adds.append(p.pid)
                    ld += p.demand
            cand = [p.pid for p in rest] + adds
            cand_w = _covered_weight(points, cand)
            if cand_w > cur_w + 1e-9:
                cur = cand
                cur_w = cand_w
                improved = True
                break
    return cur


def select_points(points: Sequence[DemandPoint],
                  capacity: int) -> List[str]:
    """贪心 + 交换改进的选点入口。"""
    return swap_improve(points, greedy_select(points, capacity), capacity)


def route_order(points: Sequence[DemandPoint], selected: Sequence[str],
                depot: Tuple[float, float]) -> List[str]:
    """架次内最近邻排序（从补给点出发）。"""
    remain = list(selected)
    order, cur = [], np.asarray(depot, dtype=float)
    while remain:
        best = min(remain, key=lambda k: float(
            np.linalg.norm(next(p for p in points if p.pid == k).xy - cur)))
        cur = next(p for p in points if p.pid == best).xy
        order.append(best)
        remain.remove(best)
    return order


def plan_missions(points: Sequence[DemandPoint],
                  cfg: CargoCfg = CargoCfg()) -> Tuple[List[str], List[Mission]]:
    """多架次规划：容量装满一架就走，回补给点装载 reload_s 后下一架次。

    返回 (全部入选点, 架次序列)。需求超出全部架次能力的点被记录跳过。
    """
    remaining = list(points)
    missions: List[Mission] = []
    all_selected: List[str] = []
    depart = 0.0
    sortie = 0
    while remaining:
        sortie += 1
        picked = select_points(remaining, cfg.capacity)
        if not picked:  # 剩余点单点超载，永远装不下
            break
        pts = [p for p in remaining if p.pid in picked]
        ordered = route_order(remaining, picked, cfg.depot)
        pmap = {p.pid: p for p in pts}
        dist, cur = 0.0, np.asarray(cfg.depot, dtype=float)
        for pid in ordered:
            nxt = pmap[pid].xy
            dist += float(np.linalg.norm(nxt - cur))
            cur = nxt
        dist += float(np.linalg.norm(np.asarray(cfg.depot, dtype=float) - cur))
        flight = dist / cfg.cruise_speed
        load = sum(p.demand for p in pts)
        missions.append(Mission(sortie, ordered, load, dist, flight,
                                depart, pts))
        all_selected.extend(ordered)
        served = {p.pid for p in pts}
        remaining = [p for p in remaining if p.pid not in served]
        depart += flight + cfg.reload_s
    return all_selected, missions


# --------------------------------------------------------------------------- #
# 绕行几何：河流禁飞带 + 桥梁通道
# --------------------------------------------------------------------------- #
def segment_crosses_band(a, b, y_lo: float, y_hi: float) -> bool:
    """直线段是否横穿水平禁飞带 [y_lo, y_hi]（两端点在带异侧）。"""
    ya, yb = float(a[1]), float(b[1])
    return (ya < y_lo and yb > y_hi) or (ya > y_hi and yb < y_lo)


def detour_via_bridge(waypoints: Sequence[Sequence[float]],
                      y_lo: float, y_hi: float,
                      bridge_x: float) -> List[Tuple[float, float]]:
    """凡横穿禁飞带的段，插入桥梁两端点 (bridge_x, y_lo/hi) 强制绕行。"""
    out: List[Tuple[float, float]] = []
    for i, wp in enumerate(waypoints):
        out.append((float(wp[0]), float(wp[1])))
        if i + 1 >= len(waypoints):
            break
        nxt = waypoints[i + 1]
        if segment_crosses_band(wp, nxt, y_lo, y_hi):
            first_below = float(wp[1]) < y_lo
            lo_pt = (float(bridge_x), y_lo)
            hi_pt = (float(bridge_x), y_hi)
            out.extend([lo_pt, hi_pt] if first_below else [hi_pt, lo_pt])
    return out
