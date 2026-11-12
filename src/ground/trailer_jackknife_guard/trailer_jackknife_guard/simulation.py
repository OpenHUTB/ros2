#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三个对照场景：无保护倒车 / 带保护倒车 / 前进自稳。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .model import TrailerParams, guard_state, hitch_rate

DT = 0.01


@dataclass
class ScenarioResult:
    name: str
    jackknifed: bool
    time_to_jackknife: float
    max_abs_phi_deg: float
    final_phi_deg: float
    min_allowed_speed: float
    max_requested_speed: float
    trace: list = field(default_factory=list)


def run_scenario(name: str, v_request: float, delta: float,
                 p: TrailerParams, duration: float = 30.0,
                 guarded: bool = False, horizon: float = 5.0) -> ScenarioResult:
    """``v_request`` 符号表示方向：负为倒车，正为前进。"""
    sign = 1.0 if v_request >= 0.0 else -1.0
    v_mag_req = abs(v_request)
    phi = 0.0
    t = 0.0
    jackknifed = False
    t_jack = float("inf")
    max_phi = 0.0
    min_allowed = float("inf")
    trace = []

    steps = int(round(duration / DT))
    decimation = max(1, int(round(0.05 / DT)))     # 每 0.1 s 评估一次保护
    allowed_now = v_mag_req
    for k in range(steps):
        if sign < 0.0 and (k % decimation == 0 or not guarded):
            allowed_now, _level, _msg = guard_state(phi, delta, v_mag_req, p, horizon)
            min_allowed = min(min_allowed, allowed_now)
        if abs(phi) >= p.jackknife_angle:
            break
        v_mag = min(v_mag_req, allowed_now) if (guarded and sign < 0.0) else v_mag_req
        v = sign * v_mag
        phi += hitch_rate(v, delta, phi, p) * DT
        t = (k + 1) * DT
        max_phi = max(max_phi, abs(phi))
        if abs(phi) >= p.jackknife_angle and not jackknifed:
            jackknifed = True
            t_jack = t
            if not guarded:
                break
        if k % 10 == 0:
            trace.append((t, math.degrees(phi), v, v_mag))

    return ScenarioResult(
        name=name,
        jackknifed=jackknifed,
        time_to_jackknife=t_jack,
        max_abs_phi_deg=math.degrees(max_phi),
        final_phi_deg=math.degrees(phi),
        min_allowed_speed=0.0 if min_allowed == float("inf") else min_allowed,
        max_requested_speed=v_mag_req,
        trace=trace,
    )


def default_scenarios(p: TrailerParams | None = None,
                      duration: float = 30.0) -> list[ScenarioResult]:
    p = p or TrailerParams()
    delta = math.radians(6.0)
    return [
        run_scenario("reverse_unguarded", -1.2, delta, p, duration,
                     guarded=False),
        run_scenario("reverse_guarded", -1.2, delta, p, duration, guarded=True),
        run_scenario("forward_stable", 1.2, delta, p, duration, guarded=False),
    ]


def format_report(results) -> str:
    lines = ["半挂车折叠风险与限速保护", "-" * 56]
    for r in results:
        lines.append("[%s]" % r.name)
        if r.jackknifed:
            lines.append("  折叠: 是，%.2f s 达到折叠角" % r.time_to_jackknife)
        else:
            lines.append("  折叠: 否")
        lines.append("  最大铰接角: %.2f°（末值 %.2f°）"
                     % (r.max_abs_phi_deg, r.final_phi_deg))
        lines.append("  请求速度: %.2f m/s，最小允许速度: %.2f m/s"
                     % (r.max_requested_speed, r.min_allowed_speed))
    return "\n".join(lines)
