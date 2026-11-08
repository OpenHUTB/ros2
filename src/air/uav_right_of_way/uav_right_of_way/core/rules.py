#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通行权规则引擎：输入两机状态，输出让路动作 + 可审计的决策理由。

四条规则（按优先级从高到低逐条适用，几何条件决定命中哪条）：

① 优先级：``priority`` 高者先行（医疗/警用等），低者让路——
   对头几何右转 30°，其余几何降高 10 m；
② 左让右（同优先级交叉相遇）：用航向叉积判左右，位于左侧的一方
   右转 30° 让行，位于右侧的一方保持航向航速；
③ 对头相遇（同优先级）：各自右转 30°；
④ 追越（同优先级、小航向差）：前机保速保向；追越机必须**从左侧**
   通过——已具左侧几何则左偏 ``overtake_lateral_m`` 追越，
   若位于前机右侧则先减速 30% 保持间隔。

动作词表：右转 / 降高 / 减速 / 保向（外加规则④自身要求的左侧追越偏置）。
每条输出都带 ``rule`` 标签与人类可读的 ``reason`` 字符串，供日志审计。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .detect import AircraftState, Threat

# ------------------------------------------------------------------ 动作词表
ACTION_TURN_RIGHT = "turn_right"        # 右转（value=角度 deg）
ACTION_DESCEND = "descend"              # 降高（value=下降量 m）
ACTION_SLOW = "slow"                    # 减速（value=减速比例 0~1）
ACTION_MAINTAIN = "maintain"            # 保向保速
ACTION_OVERTAKE_LEFT = "overtake_left"  # 左侧追越偏置（value=偏置 m）


@dataclass(frozen=True)
class RuleParams:
    """规则动作参数（对应 config/uav_right_of_way.yaml）。"""

    turn_right_deg: float = 30.0
    descend_m: float = 10.0
    slow_fraction: float = 0.30
    overtake_lateral_m: float = 20.0
    #: 判定对头的航向差阈值（cos < cos(150°)）
    headon_cos: float = -0.8660254037844387
    #: 对头还要求对方接近正前方（相对方位角 < 25°）
    headon_bearing_deg: float = 25.0
    #: 判定同向追越的航向差阈值（cos > cos(10°)）
    same_dir_cos: float = 0.984807753012208
    #: 追越左右侧判定的最小横向距离（m）
    side_eps_m: float = 3.0


@dataclass(frozen=True)
class Action:
    """一个让路动作。"""

    kind: str
    value: float = 0.0

    def describe(self) -> str:
        if self.kind == ACTION_TURN_RIGHT:
            return "右转%.0f°" % self.value
        if self.kind == ACTION_DESCEND:
            return "降高%.0fm" % self.value
        if self.kind == ACTION_SLOW:
            return "减速%.0f%%" % (self.value * 100.0)
        if self.kind == ACTION_OVERTAKE_LEFT:
            return "左侧偏置%.0fm追越" % self.value
        return "保向保速"

    def as_dict(self) -> dict:
        return dict(kind=self.kind, value=round(float(self.value), 2))


@dataclass
class Decision:
    """一次规则引擎输出（可审计）。"""

    t: float
    uid: str            # 决策主体（让路/保持的机）
    other: str          # 威胁来源机
    rule: str           # 命中的规则标签，如 "①优先级"
    action: Action
    reason: str         # 决策理由（人类可读，写日志/JSON）
    cpa_t: float = 0.0
    cpa_dist: float = 0.0

    def as_dict(self) -> dict:
        return dict(t=round(self.t, 2), uid=self.uid, other=self.other,
                    rule=self.rule, action=self.action.as_dict(),
                    reason=self.reason,
                    cpa_t=round(self.cpa_t, 2), cpa_dist=round(self.cpa_dist, 2))


def _unit(hdg: float) -> np.ndarray:
    return np.array([math.cos(hdg), math.sin(hdg)])


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def classify(own: AircraftState, other: AircraftState,
             params: RuleParams = RuleParams()) -> str:
    """两机相遇几何分类：``headon`` / ``overtake`` / ``crossing``。"""
    h_own = own.heading
    h_other = other.heading
    if h_own is None or h_other is None:
        return "crossing"
    diff = _wrap_pi(h_other - h_own)
    rel = np.asarray(other.pos[:2]) - np.asarray(own.pos[:2])
    rel_bearing = abs(_wrap_pi(math.atan2(rel[1], rel[0]) - h_own))
    if math.cos(diff) < params.headon_cos and \
            rel_bearing < math.radians(params.headon_bearing_deg):
        return "headon"
    if math.cos(diff) > params.same_dir_cos:
        return "overtake"
    return "crossing"


def decide(own: AircraftState, other: AircraftState,
           threat: Optional[Threat] = None, t: float = 0.0,
           params: RuleParams = RuleParams()) -> Decision:
    """规则引擎：给定两机状态返回决策（动作 + 规则标签 + 理由）。"""
    if threat is None:
        d = float(np.linalg.norm(np.asarray(other.pos, dtype=float)
                                 - np.asarray(own.pos, dtype=float)))
        threat = Threat(other.uid, 0.0, d)

    h_own = own.heading
    h_other = other.heading
    if h_own is None or h_other is None:
        return Decision(t, own.uid, other.uid, "—", Action(ACTION_MAINTAIN),
                        "速度过低，保持当前状态", threat.cpa_t, threat.cpa_dist)

    rel = np.asarray(other.pos[:2], dtype=float) - np.asarray(own.pos[:2], dtype=float)
    geo = classify(own, other, params)

    # ---- 规则①优先级：高者先行 ---------------------------------------
    if own.priority < other.priority:
        if geo == "headon":
            act = Action(ACTION_TURN_RIGHT, params.turn_right_deg)
            reason = ("规则①优先级：对方 %s 优先级 %d 高于本机 %d，对头相遇右转%.0f°让行"
                      % (other.uid, other.priority, own.priority,
                         params.turn_right_deg))
        else:
            act = Action(ACTION_DESCEND, params.descend_m)
            reason = ("规则①优先级：对方 %s 优先级 %d 高于本机 %d，降高%.0fm让行"
                      % (other.uid, other.priority, own.priority,
                         params.descend_m))
        return Decision(t, own.uid, other.uid, "①优先级", act, reason,
                        threat.cpa_t, threat.cpa_dist)
    if own.priority > other.priority:
        act = Action(ACTION_MAINTAIN)
        reason = ("规则①优先级：本机优先级 %d 高于对方 %s 的 %d，保持航向航速由对方让路"
                  % (own.priority, other.uid, other.priority))
        return Decision(t, own.uid, other.uid, "①优先级", act, reason,
                        threat.cpa_t, threat.cpa_dist)

    # ---- 规则③对头相遇：各自右转 --------------------------------------
    if geo == "headon":
        diff_deg = math.degrees(abs(_wrap_pi(h_other - h_own)))
        act = Action(ACTION_TURN_RIGHT, params.turn_right_deg)
        reason = ("规则③对头相遇：与 %s 航向差 %.0f°、CPA %.0fm@%.1fs，各自右转%.0f°"
                  % (other.uid, diff_deg, threat.cpa_dist, threat.cpa_t,
                     params.turn_right_deg))
        return Decision(t, own.uid, other.uid, "③对头", act, reason,
                        threat.cpa_t, threat.cpa_dist)

    # ---- 规则④追越：前机保速保向，追越机从左侧通过 ---------------------
    if geo == "overtake":
        u_other = _unit(h_other)
        own_from_other = -rel                       # 本机相对前机的位置
        cross_z = float(u_other[0] * own_from_other[1]
                        - u_other[1] * own_from_other[0])
        along = float(np.dot(own_from_other, u_other))   # >0 本机在前
        if along > 0.0:
            act = Action(ACTION_MAINTAIN)
            reason = ("规则④追越：本机为前机，保速保向，由 %s 从左侧追越"
                      "（CPA %.0fm@%.1fs）" % (other.uid, threat.cpa_dist,
                                              threat.cpa_t))
            return Decision(t, own.uid, other.uid, "④追越", act, reason,
                            threat.cpa_t, threat.cpa_dist)
        if cross_z < -params.side_eps_m:
            act = Action(ACTION_SLOW, params.slow_fraction)
            reason = ("规则④追越：须从左侧追越 %s，本机现处其右侧，先减速%.0f%%保持间隔"
                      "（CPA %.0fm@%.1fs）" % (other.uid, params.slow_fraction * 100.0,
                                              threat.cpa_dist, threat.cpa_t))
            return Decision(t, own.uid, other.uid, "④追越", act, reason,
                            threat.cpa_t, threat.cpa_dist)
        act = Action(ACTION_OVERTAKE_LEFT, params.overtake_lateral_m)
        side = "已具左侧几何" if cross_z > params.side_eps_m else "正后方"
        reason = ("规则④追越：从左侧追越 %s，左偏%.0fm保持间隔（%s，CPA %.0fm@%.1fs）"
                  % (other.uid, params.overtake_lateral_m, side,
                     threat.cpa_dist, threat.cpa_t))
        return Decision(t, own.uid, other.uid, "④追越", act, reason,
                        threat.cpa_t, threat.cpa_dist)

    # ---- 规则②交叉相遇：左让右（航向叉积判左右） -----------------------
    u_own = _unit(h_own)
    cross_z = float(u_own[0] * rel[1] - u_own[1] * rel[0])
    if cross_z < 0.0:
        act = Action(ACTION_TURN_RIGHT, params.turn_right_deg)
        reason = ("规则②交叉相遇左让右：%s 位于本机右侧（叉积 %.1f），右转%.0f°让行"
                  "（CPA %.0fm@%.1fs）" % (other.uid, cross_z,
                                           params.turn_right_deg,
                                           threat.cpa_dist, threat.cpa_t))
        return Decision(t, own.uid, other.uid, "②左让右", act, reason,
                        threat.cpa_t, threat.cpa_dist)
    act = Action(ACTION_MAINTAIN)
    reason = ("规则②交叉相遇左让右：本机位于 %s 右侧（叉积 %.1f，右方机优先），"
              "保持航向航速（CPA %.0fm@%.1fs）"
              % (other.uid, cross_z, threat.cpa_dist, threat.cpa_t))
    return Decision(t, own.uid, other.uid, "②左让右", act, reason,
                    threat.cpa_t, threat.cpa_dist)
