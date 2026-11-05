#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航线降噪优化：三种手段按代价择优，约束航程增加 ≤ 15 %。

手段（对每架飞机可组合、贪心迭代）：

1. **抬升巡航高度** +30 / +60 m —— 球面扩散直接把敏感点 Lp 拉低约
   20·log10(r2/r1)，代价是爬升投影变长、巡航段被压缩（几何不可行时跳过）；
2. **侧向平移航线** —— 把巡航段整体横移，使其与敏感点保持 ≥ 200 m 缓冲
   距离，代价是地面航程增加；
3. **起降爬升段更陡** 8° → 12° —— 缩短贴地的爬升水平投影，让无人机更快
   离开起降点附近的敏感点，代价是垂直通道指令更激进（闭环可控性由
   MuJoCo 复飞验证）。

优化目标 J = Σ(敏感点 LAeq 的能量 × 权重)（合成 dB 后报告）；
约束 = 全部航迹的地面航程增幅 ≤ 15 %。

择优策略：贪心迭代——每一轮枚举所有 (飞机, 手段) 候选，筛掉几何不可行
与违反航程约束的，按"目标改善量 / 代价"排序取最优施加，直到无候选能
再改善或达到轮数上限。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .exposure import (SensitivePoint, laeq_at_points, objective,
                       sample_track)
from .track import Flight


@dataclass
class MitigationParams:
    """降噪手段与约束参数（与 config/params.yaml 对应）。"""

    alt_raises: Tuple[float, ...] = (30.0, 60.0)
    buffer: float = 200.0
    climb_angle_base: float = 8.0
    climb_angle_steep: float = 12.0
    max_path_increase: float = 0.15
    lateral_margin: float = 10.0     # 侧移时在缓冲区外再留的裕量


@dataclass
class Action:
    """一条已施加的降噪动作。"""

    kind: str          # alt_raise / lateral_offset / steeper_climb
    uid: str
    value: float
    note: str = ""

    def describe(self) -> str:
        if self.kind == "alt_raise":
            return "%s 巡航高度抬升 %+.0f m" % (self.uid, self.value)
        if self.kind == "lateral_offset":
            return "%s 侧向平移 %.0f m 避开缓冲区" % (self.uid, self.value)
        return "%s 爬升角 %.0f°→%.0f°" % (self.uid, 8.0, self.value)


@dataclass
class Resolution:
    """一轮优化的完整结果。"""

    actions: List[Action]
    plans: Dict[str, Flight]                    # uid -> 优化后剖面
    objective_before: float                     # dB（能量加权合成）
    objective_after: float
    path_before: float                          # 地面航程合计 m
    path_after: float
    max_path_increase: float
    skipped: List[str] = field(default_factory=list)   # 不可行候选的原因

    @property
    def path_increase_ratio(self) -> float:
        return (self.path_after - self.path_before) / self.path_before

    @property
    def improvement_db(self) -> float:
        return self.objective_before - self.objective_after


# ------------------------------------------------------------------ 几何辅助
def min_point_distance(flight: Flight,
                       points: Sequence[SensitivePoint]) -> Dict[str, float]:
    """剖面全程对每个敏感点的最小三维距离（m）。"""
    pos, _dt = sample_track(flight, ds=4.0)
    out = {}
    for pt in points:
        d = np.linalg.norm(
            pos - np.array([pt.x, pt.y, 0.0]), axis=1)
        out[pt.name] = float(d.min())
    return out


def _lateral_offset_for(flight: Flight, points: Sequence[SensitivePoint],
                        params: MitigationParams) -> Optional[float]:
    """求使巡航段与所有敏感点水平距离 ≥ buffer 的带符号横移量。

    只处理所有"过近"敏感点都在航迹同一侧的情形；两侧都有敏感点时
    单次平移无法同时满足，返回 ``None``（由调用方记录原因）。
    """
    pos, _ = sample_track(flight, ds=4.0)
    xy = pos[:, :2]
    signs = set()
    offset = 0.0
    for pt in points:
        d = np.linalg.norm(xy - np.array([pt.x, pt.y]), axis=1)
        k = int(np.argmin(d))
        if d[k] >= params.buffer:
            continue
        # 最近点处航向的单位切向量
        i0, i1 = max(k - 1, 0), min(k + 1, len(xy) - 1)
        tangent = xy[i1] - xy[i0]
        norm = np.linalg.norm(tangent)
        if norm < 1e-9:
            continue
        tangent = tangent / norm
        # 叉积 z 分量决定敏感点在航迹左侧(+)还是右侧(−)；平移方向必须
        # 朝敏感点的反方向（左有敏感点就往右移）
        cross = float(tangent[0] * (pt.y - xy[k][1]) -
                      tangent[1] * (pt.x - xy[k][0]))
        sign = -1.0 if cross > 0 else 1.0
        signs.add(sign)
        offset = max(offset, params.buffer - d[k] + params.lateral_margin)
    if not signs:
        return None                      # 没有点需要侧移
    if len(signs) > 1:
        return math.nan                  # 两侧都有，单次平移无解
    return sign * offset


# ------------------------------------------------------------------ 候选生成
def _candidates(flight: Flight, params: MitigationParams
                ) -> List[Tuple[str, float, Flight]]:
    """生成 (kind, value, modified_flight) 候选（几何可行者）。"""
    cands: List[Tuple[str, float, Flight]] = []
    for raise_ in params.alt_raises:
        try:
            cands.append(("alt_raise", raise_,
                          flight.with_altitude(flight.alt + raise_)))
        except ValueError:
            # 几何不可行（航段容不下抬升后的爬升投影）：候选记为 None，
            # 由调用方按原因跳过
            cands.append(("alt_raise", raise_, None))
    try:
        cands.append(("steeper_climb", params.climb_angle_steep,
                      flight.with_climb_angle(params.climb_angle_steep)))
    except ValueError:
        cands.append(("steeper_climb", params.climb_angle_steep, None))
    return cands


def _apply_lateral(flight: Flight, points: Sequence[SensitivePoint],
                   params: MitigationParams) -> Tuple[Optional[Flight], str]:
    off = _lateral_offset_for(flight, points, params)
    if off is None:
        return None, "无敏感点进入缓冲区，无需侧移"
    if math.isnan(off):
        return None, "敏感点分布在航迹两侧，单次平移无法同时满足"
    try:
        return flight.with_lateral_offset(off), ""
    except ValueError as exc:
        return None, str(exc)


# ------------------------------------------------------------------ 主流程
def mitigate_fleet(flights: Sequence[Flight],
                   points: Sequence[SensitivePoint],
                   params: MitigationParams | None = None,
                   ds: float = 2.0,
                   max_rounds: int = 3,
                   repeat: int = 1,
                   t_obs: float | None = None) -> Resolution:
    """贪心降噪优化主入口。每架飞机每种手段至多施加一次。"""
    params = params or MitigationParams()
    plans: Dict[str, Flight] = {f.uid: f for f in flights}
    actions: List[Action] = []
    skipped: List[str] = []
    applied: Dict[str, set] = {uid: set() for uid in plans}

    def fleet_obj(mapping: Dict[str, Flight]) -> float:
        return objective(list(mapping.values()), points, ds, repeat=repeat,
                         t_obs=t_obs)

    def fleet_path(mapping: Dict[str, Flight]) -> float:
        return sum(f.path_length_ground() for f in mapping.values())

    path0 = fleet_path(plans)
    obj0 = fleet_obj(plans)

    for _round in range(max_rounds):
        best = None      # (score, uid, kind, value, new_flight, cost)
        for uid, flight in sorted(plans.items()):
            # --- 高度抬升 / 更陡爬升
            for kind, value, cand in _candidates(flight, params):
                if kind in applied[uid]:
                    continue
                if cand is None:
                    skipped.append("%s %s %.0f：航段容不下爬升投影"
                                   % (uid, kind, value))
                    continue
                trial = dict(plans)
                trial[uid] = cand
                ratio = (fleet_path(trial) - path0) / path0
                if ratio > params.max_path_increase + 1e-9:
                    skipped.append("%s %s %.0f：航程增幅 %.1f%% 超约束"
                                   % (uid, kind, value, ratio * 100))
                    continue
                obj = fleet_obj(trial)
                cost = max(fleet_path3d(trial) - fleet_path3d(plans),
                           0.0) / max(path0, 1.0)
                gain = obj0 - obj
                if gain <= 1e-6:
                    continue
                score = gain / max(cost, 0.02)
                if best is None or score > best[0]:
                    best = (score, uid, kind, value, cand, cost)
            # --- 侧向平移
            if "lateral_offset" not in applied[uid]:
                cand, why = _apply_lateral(flight, points, params)
                if cand is None:
                    if why:
                        skipped.append("%s lateral_offset：%s" % (uid, why))
                else:
                    trial = dict(plans)
                    trial[uid] = cand
                    ratio = (fleet_path(trial) - path0) / path0
                    if ratio > params.max_path_increase + 1e-9:
                        skipped.append("%s lateral_offset：航程增幅 %.1f%% 超约束"
                                       % (uid, ratio * 100))
                    else:
                        obj = fleet_obj(trial)
                        cost = max(fleet_path3d(trial) - fleet_path3d(plans),
                                   0.0) / max(path0, 1.0)
                        gain = obj0 - obj
                        if gain > 1e-6:
                            score = gain / max(cost, 0.02)
                            if best is None or score > best[0]:
                                off = _lateral_offset_for(flight, points, params)
                                best = (score, uid, "lateral_offset",
                                        0.0 if off is None or math.isnan(off)
                                        else off, cand, cost)
        if best is None:
            break
        _score, uid, kind, value, cand, cost = best
        plans[uid] = cand
        applied[uid].add(kind)
        actions.append(Action(kind, uid, value))
        obj0 = fleet_obj(plans)

    path1 = fleet_path(plans)
    base_map = {f.uid: f for f in flights}
    return Resolution(actions=actions, plans=plans,
                      objective_before=objective(list(base_map.values()),
                                                 points, ds, repeat=repeat,
                                                 t_obs=t_obs),
                      objective_after=obj0,
                      path_before=path0, path_after=path1,
                      max_path_increase=params.max_path_increase,
                      skipped=skipped)


def fleet_path3d(mapping: Dict[str, Flight]) -> float:
    return sum(f.path_length_3d() for f in mapping.values())


def laeq_table(before: Sequence[Flight], after: Sequence[Flight],
               points: Sequence[SensitivePoint], ds: float = 2.0,
               repeat: int = 1, t_obs: float | None = None) -> List[dict]:
    """优化前后各敏感点 LAeq 对比表（README/JSON 用）。"""
    lb = laeq_at_points(before, points, ds, repeat=repeat, t_obs=t_obs)
    la = laeq_at_points(after, points, ds, repeat=repeat, t_obs=t_obs)
    rows = []
    for pt in points:
        b, a = lb[pt.name], la[pt.name]
        rows.append(dict(
            point=pt.name, kind=pt.kind, weight=pt.weight,
            limit=pt.limit,
            laeq_before=round(b, 1) if math.isfinite(b) else None,
            laeq_after=round(a, 1) if math.isfinite(a) else None,
            delta_db=round(b - a, 1) if math.isfinite(b) and math.isfinite(a)
            else None,
            before_exceed=bool(math.isfinite(b) and b > pt.limit),
            after_exceed=bool(math.isfinite(a) and a > pt.limit)))
    return rows
