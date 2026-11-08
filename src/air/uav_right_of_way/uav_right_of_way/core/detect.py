#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""侵入判定：用相对运动解析求最近接近点（CPA），预测短时威胁。

本模块回答"现在要不要启动让路决策"：给定两机当前状态（位置/速度），
把双方近似为匀速直线运动，相对运动也是匀速直线，最近接近可以解析求出：

    d(t) = |Δp + Δv·t|,   t_cpa = -Δp·Δv / |Δv|²

当 ``0 < t_cpa ≤ horizon`` 且预测 CPA 距离 < ``cpa_threshold`` 时触发决策。
与参考实现 uav_deconflict 的 ``conflict.py`` 同思路（相对运动解析、无采样
步长），但这里用在**在线**判定上：输入是当前一帧状态而不是整条计划航迹。

约定：``t_cpa ≤ 0``（正在远离）不算威胁——追越通过后两机必然逐渐拉开，
不能让"还在 40 m 内但正在分离"的状态永远触发决策。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DetectParams:
    """侵入判定参数（对应 config/uav_right_of_way.yaml）。"""

    horizon_s: float = 8.0          # 预测时域（s）
    cpa_threshold_m: float = 40.0   # CPA 距离阈值（m）

    def __post_init__(self) -> None:
        if self.horizon_s <= 0.0 or self.cpa_threshold_m <= 0.0:
            raise ValueError("horizon 与 cpa_threshold 必须为正: %r" % (self,))


@dataclass
class AircraftState:
    """一帧机体状态：位置 / 速度（世界系 ENU，z 向上）+ 任务优先级。"""

    uid: str
    pos: np.ndarray                         # (3,) m
    vel: np.ndarray                         # (3,) m/s
    priority: int = 0
    label: str = ""

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.vel))

    @property
    def heading(self) -> Optional[float]:
        """航向角（rad，从 x 东逆时针）；速度过低时返回 None。"""
        if float(np.hypot(self.vel[0], self.vel[1])) < 1e-6:
            return None
        return float(math.atan2(self.vel[1], self.vel[0]))

    def as_dict(self) -> dict:
        return dict(uid=self.uid,
                    pos=[round(float(v), 2) for v in self.pos],
                    vel=[round(float(v), 2) for v in self.vel],
                    priority=self.priority)


@dataclass
class Threat:
    """一次预测到的接近威胁。"""

    other: str          # 对方机号
    cpa_t: float        # 预测最近接近时刻（相对当前，已截断到 horizon）
    cpa_dist: float     # 预测最近接近距离（m）

    def as_dict(self) -> dict:
        return dict(other=self.other,
                    cpa_t=round(self.cpa_t, 2),
                    cpa_dist=round(self.cpa_dist, 2))


def cpa(p_rel: np.ndarray, v_rel: np.ndarray,
        horizon: float) -> Tuple[float, float]:
    """相对运动解析 CPA：返回 (t_cpa, 最近距离)。

    ``t_cpa`` 已截断到 ``[0, horizon]``；相对静止时返回 (0, 当前距离)。
    """
    p_rel = np.asarray(p_rel, dtype=float)
    v_rel = np.asarray(v_rel, dtype=float)
    a = float(np.dot(v_rel, v_rel))
    if a <= 1e-9:
        return 0.0, float(np.linalg.norm(p_rel))
    t = -float(np.dot(p_rel, v_rel)) / a
    t = min(max(t, 0.0), horizon)
    return t, float(np.linalg.norm(p_rel + v_rel * t))


def predict_threat(own: AircraftState, other: AircraftState,
                   params: DetectParams = DetectParams()) -> Optional[Threat]:
    """预测本机与另一机的接近威胁；不满足触发条件返回 ``None``。"""
    p_rel = np.asarray(other.pos, dtype=float) - np.asarray(own.pos, dtype=float)
    v_rel = np.asarray(other.vel, dtype=float) - np.asarray(own.vel, dtype=float)
    a = float(np.dot(v_rel, v_rel))
    if a <= 1e-9:
        d = float(np.linalg.norm(p_rel))
        return Threat(other.uid, 0.0, d) if d < params.cpa_threshold_m else None
    t_unc = -float(np.dot(p_rel, v_rel)) / a
    if t_unc <= 0.0:
        # 正在远离：不算威胁（否则追越通过后永远触发）
        return None
    t = min(t_unc, params.horizon_s)
    d = float(np.linalg.norm(p_rel + v_rel * t))
    if d < params.cpa_threshold_m:
        return Threat(other.uid, t, d)
    return None


def fleet_threats(states: Sequence[AircraftState],
                  params: DetectParams = DetectParams()
                  ) -> Dict[Tuple[str, str], Threat]:
    """机队两两检测，返回 {(uid_a, uid_b): Threat}（a 按字典序在 b 前）。"""
    out: Dict[Tuple[str, str], Threat] = {}
    items = sorted(states, key=lambda s: s.uid)
    for i, a in enumerate(items):
        for b in items[i + 1:]:
            th = predict_threat(a, b, params)
            if th is not None:
                out[(a.uid, b.uid)] = th
    return out


def threats_against(own: AircraftState,
                    others: Sequence[AircraftState],
                    params: DetectParams = DetectParams()) -> Dict[str, Threat]:
    """以本机视角对所有其他机做威胁检测，返回 {对方机号: Threat}。"""
    out: Dict[str, Threat] = {}
    for o in others:
        if o.uid == own.uid:
            continue
        th = predict_threat(own, o, params)
        if th is not None:
            out[o.uid] = th
    return out
