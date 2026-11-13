#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""COLREGs 会遇局面判定与避让决策（纯 Python，可单测）。

约定（右手系、角度用弧度）：

* 本船位置 ``p0``、航向 ``chi0``、速度 ``v0``；目标船 ``p1``、``chi1``、``v1``；
* 相对方位 ``beta = wrap(atan2(dy, dx) - chi0)``，右正左负；
* 先算 CPA/TCPA，再分类，避免「远距离高速目标被当成安全」：

  - ``opening``：TCPA < 0，两船正在拉开，无碰撞危险；
  - ``clear``：CPA > 安全距离 **且** TCPA > 预警时间；
  - ``overtaken``：目标在正横后方（|beta| > 112.5°），本船为直航船；
  - ``head_on``：目标在正前方 ±5° 且航向接近相反（>= 170°）；
  - ``overtaking_own``：目标在前方 ±45°、同向（航向差 <= 22.5°）且本船更快，
    按 Rule 13 本船是让路船；
  - ``crossing_give_way``：目标在右舷，本船让路；
  - ``crossing_stand_on``：目标在左舷，本船直航。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

HEAD_ON_BEARING = math.radians(5.0)
AFT_BEAM = math.radians(112.5)
HEAD_ON_HEADING = math.radians(170.0)
OVERTAKE_HEADING = math.radians(22.5)
OVERTAKE_BEARING = math.radians(45.0)

STARBOARD_HEAD_ON = math.radians(30.0)
STARBOARD_CROSSING = math.radians(45.0)
STARBOARD_OVERTAKE = math.radians(30.0)
STARBOARD_EMERGENCY = math.radians(30.0)


@dataclass
class Thresholds:
    cpa_safe: float = 500.0
    cpa_warn: float = 300.0
    tcpa_warn: float = 180.0
    tcpa_danger: float = 60.0


DEFAULT_THRESHOLDS = Thresholds()


def wrap_to_pi(angle: float) -> float:
    value = math.fmod(float(angle) + math.pi, 2.0 * math.pi)
    if value <= 0.0:
        value += 2.0 * math.pi
    return value - math.pi


@dataclass
class ShipState:
    x: float
    y: float
    heading: float
    speed: float


@dataclass
class Encounter:
    bearing_deg: float
    aspect: str
    cpa_m: float
    tcpa_s: float
    range_m: float
    opening: bool = False


@dataclass
class Decision:
    action: str
    course_offset_deg: float
    speed_scale: float
    urgency: str
    rule: str


def relative_geometry(own: ShipState, other: ShipState,
                      th: Thresholds | None = None) -> Encounter:
    th = th or DEFAULT_THRESHOLDS
    dx, dy = other.x - own.x, other.y - own.y
    rng = math.hypot(dx, dy)
    bearing = wrap_to_pi(math.atan2(dy, dx) - own.heading)

    vx = other.speed * math.cos(other.heading) - own.speed * math.cos(own.heading)
    vy = other.speed * math.sin(other.heading) - own.speed * math.sin(own.heading)
    vv = vx * vx + vy * vy

    opening = False
    if vv < 1e-9:
        tcpa = float("inf")
        cpa = rng
    else:
        tcpa_raw = -(dx * vx + dy * vy) / vv
        if tcpa_raw < 0.0:
            # TCPA < 0：最近点在过去，说明两船正在拉开
            opening = True
            tcpa = float("inf")
            cpa = rng
        else:
            tcpa = tcpa_raw
            cpa = math.hypot(dx + vx * tcpa, dy + vy * tcpa)

    rel_heading = abs(wrap_to_pi(other.heading - own.heading))

    if opening:
        aspect = "opening"
    elif cpa > th.cpa_safe and tcpa > th.tcpa_warn:
        aspect = "clear"
    elif abs(bearing) >= AFT_BEAM:
        aspect = "overtaken"
    elif abs(bearing) <= HEAD_ON_BEARING and rel_heading >= HEAD_ON_HEADING:
        aspect = "head_on"
    elif (rel_heading <= OVERTAKE_HEADING and own.speed > other.speed
          and abs(bearing) <= OVERTAKE_BEARING and tcpa < float("inf")):
        aspect = "overtaking_own"
    elif bearing > 0.0:
        aspect = "crossing_give_way"
    else:
        aspect = "crossing_stand_on"

    return Encounter(math.degrees(bearing), aspect, cpa, tcpa, rng, opening)


def _urgency(enc: Encounter, th: Thresholds) -> str:
    if enc.aspect in ("clear", "opening"):
        return "none"
    if enc.cpa_m <= th.cpa_warn or enc.tcpa_s <= th.tcpa_danger:
        return "danger"
    if enc.cpa_m <= th.cpa_safe or enc.tcpa_s <= th.tcpa_warn:
        return "warning"
    return "none"


def decide(enc: Encounter, th: Thresholds | None = None) -> Decision:
    """按会遇局面给出建议动作（course offset 右正左负）。"""
    th = th or DEFAULT_THRESHOLDS
    urgency = _urgency(enc, th)

    if enc.aspect in ("clear", "opening"):
        return Decision("hold", 0.0, 1.0, "none", "-")

    if enc.aspect == "head_on":
        if urgency == "none":
            return Decision("hold", 0.0, 1.0, "none", "Rule 14（监视）")
        return Decision("alter_starboard", math.degrees(STARBOARD_HEAD_ON), 1.0,
                        urgency, "Rule 14")

    if enc.aspect == "crossing_give_way":
        if urgency == "none":
            return Decision("hold", 0.0, 1.0, "none", "Rule 15（监视）")
        return Decision("alter_starboard", math.degrees(STARBOARD_CROSSING), 0.9,
                        urgency, "Rule 15/16")

    if enc.aspect == "overtaking_own":
        if urgency == "none":
            return Decision("hold", 0.0, 1.0, "none", "Rule 13（监视）")
        return Decision("alter_starboard", math.degrees(STARBOARD_OVERTAKE), 0.9,
                        urgency, "Rule 13")

    if enc.aspect in ("crossing_stand_on", "overtaken"):
        if urgency == "danger":
            return Decision("emergency_starboard",
                            math.degrees(STARBOARD_EMERGENCY), 1.0,
                            urgency, "Rule 17(b)")
        return Decision("hold", 0.0, 1.0, urgency, "Rule 17(a)")

    return Decision("hold", 0.0, 1.0, urgency, "-")


def apply_course_offset(state: ShipState, offset_deg: float,
                        speed_scale: float = 1.0) -> ShipState:
    return ShipState(state.x, state.y,
                     wrap_to_pi(state.heading + math.radians(offset_deg)),
                     state.speed * speed_scale)
