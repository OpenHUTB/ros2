#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Release timing: analytic release window along a straight approach leg.

Single-line summary: given the approach segment through a target, solve the
arc-length interval where releasing lands within acceptance radius; entering
the window fires the drop command, infeasible windows yield slow-down or
detour advice.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .ballistics import (BallisticsCfg, ReleasePlan, ReleaseWindow,
                         aim_release_point, impact_point, release_window)


@dataclass(frozen=True)
class TimingVerdict:
    """一次“当前是否该投”的判定结果（供节点与闭环共用）。"""

    release_now: bool
    predicted_impact: Optional[np.ndarray]
    window: ReleaseWindow

    def as_dict(self) -> dict:
        d = dict(release_now=self.release_now,
                 window=self.window.as_dict())
        if self.predicted_impact is not None:
            d["predicted_impact_m"] = [round(float(self.predicted_impact[0]), 2),
                                       round(float(self.predicted_impact[1]), 2)]
        return d


def remaining_distance(pos, seg_a, seg_b) -> float:
    """当前位置沿段方向的剩余弧长（到段终点 b 的投影距离）。"""
    p = np.asarray(pos, dtype=float)
    a = np.asarray(seg_a, dtype=float)
    b = np.asarray(seg_b, dtype=float)
    u = b - a
    seg_len = float(np.linalg.norm(u))
    if seg_len < 1e-9:
        return 0.0
    u /= seg_len
    return float(np.dot(b - p, u))


def evaluate_drop(pos, vel, seg_a, seg_b, target, plan: ReleasePlan,
                  released: bool = False,
                  cfg: BallisticsCfg = BallisticsCfg()) -> TimingVerdict:
    """给定当前状态判定是否进入投放区间。

    判定纯解析：当前位置沿迹弧长落入窗口且未投过 → release_now；
    预计落点用真实释放状态 + 弹道解析解。
    """
    win = release_window(seg_a, seg_b, target, plan, cfg)
    if released or not win.feasible:
        return TimingVerdict(False, None, win)
    s = _arc_length(pos, seg_a, seg_b)
    if s < win.s_enter or s > win.s_exit:
        return TimingVerdict(False, None, win)
    impact = impact_point(pos, vel, plan)
    return TimingVerdict(True, impact, win)


def _arc_length(pos, seg_a, seg_b) -> float:
    p = np.asarray(pos, dtype=float)
    a = np.asarray(seg_a, dtype=float)
    u = np.asarray(seg_b, dtype=float) - a
    seg_len = float(np.linalg.norm(u))
    if seg_len < 1e-9:
        return 0.0
    return float(np.dot(p - a, u / seg_len))


def check_leg(seg_a, seg_b, target, plan: ReleasePlan,
              cfg: BallisticsCfg = BallisticsCfg()) -> ReleaseWindow:
    """航段预检：提前量所需水平距离是否 ≤ 段长，不满足给绕行/降速建议。"""
    return release_window(seg_a, seg_b, target, plan, cfg)


def ideal_release_state(target, seg_a, seg_b,
                        plan: ReleasePlan) -> Tuple[np.ndarray, np.ndarray]:
    """理想释放点及其速度（测试与解析校验用）。"""
    a = np.asarray(seg_a, dtype=float)
    b = np.asarray(seg_b, dtype=float)
    u = b - a
    u = u / max(float(np.linalg.norm(u)), 1e-9)
    vel_xy = plan.speed * u
    return aim_release_point(target, vel_xy, plan), vel_xy
