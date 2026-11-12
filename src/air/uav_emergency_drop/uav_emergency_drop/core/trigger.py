#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Release timing: real-state release decision along an approach leg.

Single-line summary: given the approach segment through a target, check the
real position/velocity/altitude against the plan, extrapolate the impact
point and release when it is confidently inside the acceptance radius;
infeasible geometry yields slow-down or detour advice.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .ballistics import (BallisticsCfg, ReleasePlan, ReleaseWindow,
                         aim_release_point, fall_time_from_state,
                         impact_point_state, release_window)

#: 地面高度（与 flyer 的 GROUND_Z 一致，odom 的 z 以此为海拔零点）
GROUND_LEVEL = 0.05


@dataclass(frozen=True)
class TimingVerdict:
    """一次“当前是否该投”的判定结果（供节点与闭环共用）。"""

    release_now: bool
    predicted_impact: Optional[np.ndarray]
    window: ReleaseWindow
    reason: str = ""      # release_now=False 时给出具体原因

    def as_dict(self) -> dict:
        d = dict(release_now=self.release_now,
                 window=self.window.as_dict())
        if self.predicted_impact is not None:
            d["predicted_impact_m"] = [round(float(self.predicted_impact[0]), 2),
                                       round(float(self.predicted_impact[1]), 2)]
        if self.reason:
            d["reason"] = self.reason
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


def _xy(v) -> np.ndarray:
    """取水平分量（输入可为 2D 或带高度的 3D）。"""
    return np.asarray(v, dtype=float)[:2]


def evaluate_drop(pos, vel, seg_a, seg_b, target, plan: ReleasePlan,
                  released: bool = False,
                  cfg: BallisticsCfg = BallisticsCfg()) -> TimingVerdict:
    """给定当前状态判定是否该投。

    解析窗口（release_window）只做规划预检与建议：切角后进入段变短时
    s_exit 会越过段尾，把"段裁剪不可行"当投放闸门会漏掉本可命中的机会。
    实际放行只看真实三维状态外推的落点：状态全维有限（含竖直分量）、
    离地且可达地面、有前进速度且大致沿进入段方向，最后由实际高度与
    竖直速度解出的下落时间外推落点，对照接受半径判定。高度不再是
    独立的宽松门槛——掉高或竖直速度异常都会如实反映到落点偏差里，
    与接受半径同一把尺子。
    触发时机选在解析理想释放点 s_ideal（提前量处，预测落点误差最小的
    时刻）——此前"一进接受半径就投"会在窗口边缘误差贴着 accept 处松手，
    白白吃满裕量；若到达 s_ideal 时误差仍超限，则继续按同一判据观察，
    误差回到接受半径内再投（不盲投、不漏投）。
    """
    win = release_window(seg_a, seg_b, target, plan, cfg)
    if released:
        return TimingVerdict(False, None, win, "already released")

    p = np.asarray(pos, dtype=float)
    v = np.asarray(vel, dtype=float)
    # 三维状态有限性：竖直分量 NaN/inf 同样拒绝（此前只查水平两维）
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(v))):
        return TimingVerdict(False, None, win, "non-finite state")

    # 实际高度与竖直速度参与下落时间：z 轴向上，先减去地面高度
    if p.ndim >= 1 and len(p) >= 3:
        z_rel = float(p[2]) - GROUND_LEVEL
        vz = float(v[2]) if (v.ndim >= 1 and len(v) >= 3) else 0.0
        if z_rel <= 0.0:
            return TimingVerdict(False, None, win,
                                 "altitude %.2f m at/below ground" % z_rel)
        try:
            t_fall = fall_time_from_state(z_rel, vz, cfg)
        except ValueError:
            return TimingVerdict(False, None, win,
                                 "state never reaches ground (vz %.1f m/s)" % vz)
    else:
        # 纯 2D 输入（老测试/规划层）：退回计划高度的平飞假设
        t_fall = plan.t_fall

    speed = float(np.linalg.norm(_xy(v)))
    if speed < 0.5 * plan.speed:
        return TimingVerdict(False, None, win,
                             "speed %.2f m/s too low" % speed)
    if speed > 1.5 * plan.speed:
        return TimingVerdict(False, None, win,
                             "speed %.2f m/s too high" % speed)
    u = np.asarray(seg_b, dtype=float)[:2] - np.asarray(seg_a, dtype=float)[:2]
    u = u / max(float(np.linalg.norm(u)), 1e-9)
    heading = float(np.dot(_xy(v) / speed, u))
    if heading < 0.3:
        return TimingVerdict(False, None, win,
                             "heading off approach (cos %.2f)" % heading)

    impact = impact_point_state(
        np.array([p[0], p[1], z_rel if (p.ndim >= 1 and len(p) >= 3) else plan.h]),
        np.array([v[0], v[1], v[2] if (v.ndim >= 1 and len(v) >= 3) else 0.0]),
        plan.wind, cfg)
    if not np.all(np.isfinite(impact)):
        return TimingVerdict(False, None, win, "non-finite impact")
    err = float(np.linalg.norm(impact - np.asarray(target, dtype=float)[:2]))
    if err > cfg.accept_radius:
        return TimingVerdict(
            False, np.asarray(impact, dtype=float), win,
            "impact %.2f m off target > accept %.1f m" % (err, cfg.accept_radius))
    if _arc_length(_xy(pos), seg_a, seg_b) < win.s_ideal:
        return TimingVerdict(
            False, np.asarray(impact, dtype=float), win,
            "impact %.2f m off target, hold until ideal release point" % err)
    return TimingVerdict(True, impact, win)  # 到达理想释放点且落点达标：投放


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
