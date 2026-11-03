#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""评估指标：最小间隔时间序列、间隔裕度比、延误与到达偏差。

两类数据源：

* **计划**（``Plan``）—— 解析航迹，用于"解脱前后"的对比，采样步长 0.2 s；
* **真飞**（``SimResult``）—— MuJoCo 刚体积分出来的位置，用于验证消解结论。

间隔沿用圆柱模型：只有垂直间隔已经小于阈值时，水平间隔才有意义。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .conflict import Separation, detect
from .plan import Plan
from .sim import SimResult


def plan_separation_series(plans: Sequence[Plan], sep: Separation,
                           dt: float = 0.2) -> Tuple[np.ndarray, np.ndarray]:
    """按计划航迹采样，返回 (times, 最小水平间隔)。无有效样本处为 inf。"""
    if not plans:
        return np.array([]), np.array([])
    t_end = max(p.end_time for p in plans)
    times = np.arange(0.0, t_end, dt)
    out = np.full(times.shape, np.inf)
    for k, t in enumerate(times):
        pos = {}
        for p in plans:
            q = p.position_at(t)
            if q is not None:
                pos[p.uid] = q
        best = math.inf
        names = sorted(pos)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = pos[a] - pos[b]
                if abs(d[2]) >= sep.vertical:
                    continue
                best = min(best, float(math.hypot(d[0], d[1])))
        out[k] = best
    return times, out


def truth_separation_series(result: SimResult,
                            sep: Separation) -> Tuple[np.ndarray, np.ndarray]:
    """按真飞轨迹逐时刻求机队最小水平间隔（仅在垂直间隔内统计）。"""
    times = result.times
    names = sorted(result.pos)
    out = np.full(times.shape, np.inf)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            d = result.pos[a] - result.pos[b]
            h = np.linalg.norm(d[:, :2], axis=1)
            v = np.abs(d[:, 2])
            out = np.minimum(out, np.where(v < sep.vertical, h, np.inf))
    return times, out


def clearance_ratio(plans: Sequence[Plan], sep: Separation,
                    dt: float = 0.2) -> float:
    """最小间隔裕度比：max(h/sep_h, v/sep_v) 的全程最小值。

    ≥ 1 表示满足间隔标准；< 1 表示侵入。比"是否冲突"更适合做量化对比，
    因为它能区分"刚刚擦边"与"完全重合"。
    """
    if not plans:
        return math.inf
    t_end = max(p.end_time for p in plans)
    times = np.arange(0.0, t_end, dt)
    best = math.inf
    for t in times:
        pos = {p.uid: p.position_at(t) for p in plans}
        pos = {k: v for k, v in pos.items() if v is not None}
        names = sorted(pos)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = pos[a] - pos[b]
                h = float(math.hypot(d[0], d[1]))
                ratio = max(h / sep.horizontal, abs(float(d[2])) / sep.vertical)
                best = min(best, ratio)
    return best


def conflict_stats(plans: Sequence[Plan], sep: Separation) -> dict:
    cons = detect(plans, sep)
    return dict(conflicts=len(cons),
                conflict_pairs=len({c.pair for c in cons}),
                min_h=(min((c.min_h for c in cons), default=math.inf)),
                min_v=(min((c.min_v for c in cons), default=math.inf)),
                total_duration=round(sum(c.duration for c in cons), 1))


def summarize(scenario_name: str,
              baseline: Sequence[Plan],
              resolved: Sequence[Plan],
              sim_base: Optional[SimResult] = None,
              sim_res: Optional[SimResult] = None,
              sep: Optional[Separation] = None) -> dict:
    """汇总一个场景的关键指标，供报告与表格使用。"""
    sep = sep or Separation()
    base_ids = {p.uid: p for p in baseline}
    res_ids = {p.uid: p for p in resolved}
    delays = {uid: res_ids[uid].end_time - base_ids[uid].end_time for uid in base_ids}
    # 高度层下压会让某架提前到达；延误只累计正值，提前量单独统计，
    # 否则正负相抵会把"总延误"算得比单机最大延误还小。
    advance = {uid: d for uid, d in delays.items() if d < 0}
    actions = {uid: (res_ids[uid].alt != base_ids[uid].alt,
                     abs(res_ids[uid].path_length_xy() - base_ids[uid].path_length_xy()))
               for uid in base_ids}

    out = dict(
        scenario=scenario_name,
        sep=dict(horizontal=sep.horizontal, vertical=sep.vertical),
        before=conflict_stats(baseline, sep),
        after=conflict_stats(resolved, sep),
        delay_total=round(sum(d for d in delays.values() if d > 0), 1),
        advance_total=round(-sum(advance.values()), 1),
        delay_max=round(max(max(delays.values()), 0.0), 1),
        alt_changes=sum(1 for v in actions.values() if v[0]),
        lateral_extra_m=round(sum(v[1] for v in actions.values()), 1),
        delays={k: round(v, 1) for k, v in delays.items()},
    )
    if sim_base is not None:
        mb = sim_base.min_separation(sep)
        out["truth_before"] = dict(
            min_h=round(mb["min_h"], 2), pair=mb["pair"], t=round(mb["t"], 1),
            violations=len(sim_base.violations(sep)))
    if sim_res is not None:
        mr = sim_res.min_separation(sep)
        out["truth_after"] = dict(
            min_h=round(mr["min_h"], 2), pair=mr["pair"], t=round(mr["t"], 1),
            violations=len(sim_res.violations(sep)),
            arrival_error_max=round(max(sim_res.final_error.values()), 2),
            track_error_max=round(max(sim_res.max_track_error.values()), 2))
    return out
