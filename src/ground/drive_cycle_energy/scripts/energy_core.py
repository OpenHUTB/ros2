#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""工况能耗与续航核算（纯标准库，不依赖 ROS / CARLA / numpy）。

问题：一段行驶工况里，电池到底把多少能量花在滚阻、风阻和加减速上？
再生制动能收回多少？满电能跑多远？

模型（纵向动力学，纵向加速度 a，坡度忽略）：
    驱动力  F = m·a + m·g·c_rr + 0.5·ρ·Cd·A·v²
    滚阻力  F_rr = m·g·c_rr               （与车速无关）
    风阻力  F_aero = 0.5·ρ·Cd·A·v²        （与车速平方成正比）
    回收    E_regen = η · ∫|F_brake|·v dt  （只在减速时发生，η 是回收效率）
    续航    range = 电池容量 / 每公里净耗电

核心恒等式（正是第一条断言的依据）：
    E_drive − E_brake = E_roll + E_aero + ΔKE
    ⇔ E_drive = E_roll + E_aero + E_brake + ΔKE
其中 ΔKE 是整段工况的动能净变化（起终点车速相同则为零）。
滚阻与风阻都能写成速度的幂函数，因此这一分解可以逐项积出来再对账。
"""

from __future__ import division, print_function, unicode_literals

ASSERT_NAMES = (u"能量分解", u"回收上限", u"续航自洽", u"风阻上升")


class EnergyError(ValueError):
    """参数或工况非法。"""


def _positive(name, value):
    if value is None or value <= 0:
        raise EnergyError(u"%s 必须大于 0，收到 %r" % (name, value))
    return value


def _non_negative(name, value):
    if value is None or value < 0:
        raise EnergyError(u"%s 不能为负，收到 %r" % (name, value))
    return value


class VehicleSpec(object):
    """整车与电池参数。"""

    def __init__(self, mass=1800.0, c_rr=0.012, rho=1.2, cd=0.31, area=2.2,
                 gravity=9.81, battery_kwh=60.0, regen_eff=0.6):
        self.mass = _positive(u"质量", mass)
        self.c_rr = _non_negative(u"滚阻系数", c_rr)
        self.rho = _positive(u"空气密度", rho)
        self.cd = _positive(u"风阻系数", cd)
        self.area = _positive(u"迎风面积", area)
        self.gravity = _positive(u"重力加速度", gravity)
        self.battery_kwh = _positive(u"电池容量", battery_kwh)
        if regen_eff < 0 or regen_eff > 1:
            raise EnergyError(u"回收效率必须在 [0, 1]，收到 %r" % (regen_eff,))
        self.regen_eff = regen_eff

    # ---- 阻力 ----
    def rolling_force(self):
        return self.mass * self.gravity * self.c_rr

    def aero_force(self, speed):
        return 0.5 * self.rho * self.cd * self.area * speed * speed

    def resistance(self, speed):
        return self.rolling_force() + self.aero_force(speed)

    def traction_force(self, speed, accel):
        return self.mass * accel + self.resistance(speed)

    def kinetic(self, speed):
        return 0.5 * self.mass * speed * speed

    def battery_joules(self):
        return self.battery_kwh * 3.6e6

    def __repr__(self):
        return ("VehicleSpec(mass=%.1f, c_rr=%.3f, cd=%.3f, area=%.2f, "
                "battery_kwh=%.1f, regen_eff=%.2f)"
                % (self.mass, self.c_rr, self.cd, self.area,
                   self.battery_kwh, self.regen_eff))


def parse_segments(segments):
    """把 [(秒, 起始 km/h, 结束 km/h), ...] 规整成 [(t, v0, v1), ...]（m/s）。"""
    out = []
    for i, seg in enumerate(segments):
        if len(seg) != 3:
            raise EnergyError(u"第 %d 段必须是 (秒, 起始, 结束)，收到 %r" % (i, seg))
        hold, v0, v1 = seg
        if hold <= 0:
            raise EnergyError(u"第 %d 段时长必须大于 0，收到 %r" % (i, hold))
        if v0 < 0 or v1 < 0:
            raise EnergyError(u"第 %d 段车速不能为负，收到 %r" % (i, seg))
        out.append((float(hold), float(v0) / 3.6, float(v1) / 3.6))
    if not out:
        raise EnergyError(u"工况至少要有一段")
    return out


def simulate(spec, segments, dt=0.05):
    """按固定步长（中点法）积分一段工况，返回能量与统计量。"""
    segs = parse_segments(segments)
    if dt <= 0:
        raise EnergyError(u"步长必须大于 0，收到 %r" % (dt,))

    drive_j = 0.0
    brake_j = 0.0
    roll_j = 0.0
    aero_j = 0.0
    ke_j = 0.0
    distance = 0.0
    duration = 0.0
    v_max = 0.0

    for hold, v0, v1 in segs:
        accel = (v1 - v0) / hold
        steps = int(round(hold / dt)) or 1
        step = hold / steps
        for i in range(steps):
            v = v0 + accel * (i + 0.5) * step
            if v < 0:
                v = 0.0
            force = spec.traction_force(v, accel)
            work = force * v * step
            if work >= 0:
                drive_j += work
            else:
                brake_j += -work
            roll_j += spec.rolling_force() * v * step
            aero_j += spec.aero_force(v) * v * step
            ke_j += spec.mass * accel * v * step
            distance += v * step
            v_max = max(v_max, v)
        duration += hold

    regen_j = spec.regen_eff * brake_j
    net_j = drive_j - regen_j
    return {
        "drive_j": drive_j,
        "brake_j": brake_j,
        "regen_j": regen_j,
        "roll_j": roll_j,
        "aero_j": aero_j,
        "ke_j": ke_j,
        "net_j": net_j,
        "distance_m": distance,
        "duration_s": duration,
        "v_max": v_max,
    }


def per_100km(result):
    """每 100 km 净耗电（kWh）。"""
    if result["distance_m"] <= 0:
        raise EnergyError(u"工况没有行驶距离，无法折算百公里能耗")
    return result["net_j"] / 3.6e6 / (result["distance_m"] / 100000.0)


def range_km(spec, result):
    """满电续航（km）。"""
    kwh_per_100 = per_100km(result)
    if kwh_per_100 <= 0:
        raise EnergyError(u"净耗电不为正，续航无意义")
    return spec.battery_kwh / kwh_per_100 * 100.0


def share(result, key):
    """某一项能量占驱动能量的比例。"""
    if result["drive_j"] <= 0:
        raise EnergyError(u"驱动能量不为正，无法计算占比")
    return result[key] / result["drive_j"]


def check_energy(spec, result, breakdown_tolerance=0.02, regen_limit=1.0,
                 range_tolerance=0.02, aero_rising=True):
    """四条断言：分解自洽、回收上限、续航自洽、风阻随车速上升。"""
    asserts = []

    total = (result["roll_j"] + result["aero_j"] + result["brake_j"]
             + result["ke_j"])
    if result["drive_j"] <= 0:
        rel = 1.0
    else:
        rel = abs(total - result["drive_j"]) / result["drive_j"]
    asserts.append({
        "type": ASSERT_NAMES[0],
        "passed": bool(rel <= breakdown_tolerance + 1e-12),
        "detail": (u"滚阻 %.1f + 风阻 %.1f + 制动 %.1f + 动能 %.1f = %.1f kJ，"
                   u"驱动 %.1f kJ（相对偏差 %.4f，容差 %.2f）"
                   % (result["roll_j"] / 1000.0, result["aero_j"] / 1000.0,
                      result["brake_j"] / 1000.0, result["ke_j"] / 1000.0,
                      total / 1000.0, result["drive_j"] / 1000.0,
                      rel, breakdown_tolerance)),
    })

    if result["brake_j"] <= 0:
        ratio = 0.0
    else:
        ratio = result["regen_j"] / result["brake_j"]
    asserts.append({
        "type": ASSERT_NAMES[1],
        "passed": bool(ratio <= regen_limit + 1e-12),
        "detail": (u"回收 %.1f kJ / 制动 %.1f kJ = %.4f（上限 %.2f，η=%.2f）"
                   % (result["regen_j"] / 1000.0, result["brake_j"] / 1000.0,
                      ratio, regen_limit, spec.regen_eff)),
    })

    try:
        kwh_per_100 = per_100km(result)
        implied = spec.battery_kwh / kwh_per_100 * 100.0
        rel_range = abs(implied - range_km(spec, result)) / max(implied, 1e-9)
        range_ok = rel_range <= range_tolerance + 1e-12
        range_detail = (u"百公里 %.3f kWh，电池 %.1f kWh → 续航 %.1f km（相对偏差 %.4f）"
                        % (kwh_per_100, spec.battery_kwh, implied, rel_range))
    except EnergyError as exc:
        range_ok = False
        range_detail = u"无法折算续航：%s" % (exc,)
    asserts.append({
        "type": ASSERT_NAMES[2],
        "passed": bool(range_ok),
        "detail": range_detail,
    })

    aero_slow = spec.aero_force(10.0)
    aero_fast = spec.aero_force(30.0)
    rising = aero_fast > aero_slow
    asserts.append({
        "type": ASSERT_NAMES[3],
        "passed": bool(rising if aero_rising else (not rising)),
        "detail": (u"10 m/s 风阻 %.1f N，30 m/s 风阻 %.1f N（期望%s）"
                   % (aero_slow, aero_fast, u"上升" if aero_rising else u"不上升")),
    })
    return asserts


def render_energy(result, width=42):
    """按比例画能量条。"""
    rows = [
        (u"滚阻", result["roll_j"]),
        (u"风阻", result["aero_j"]),
        (u"制动", result["brake_j"]),
        (u"动能", result["ke_j"]),
        (u"驱动", result["drive_j"]),
        (u"回收", result["regen_j"]),
    ]
    biggest = max([v for _, v in rows] + [1.0])
    lines = [u"能量构成（驱动 = 满条，其余按比例）", "-" * width]
    for label, value in rows:
        cells = int(round(max(value, 0.0) / biggest * width))
        lines.append(u"%s|%s| %8.1f kJ" % (label, "#" * cells, value / 1000.0))
    lines.append("-" * width)
    return "\n".join(lines)


def format_report(spec, result, asserts):
    """把上面的一切排成人能读的报告。"""
    lines = []
    lines.append(u"工况：%.1f s，行驶 %.1f m，最高 %.1f km/h"
                 % (result["duration_s"], result["distance_m"],
                    result["v_max"] * 3.6))
    lines.append(u"整车：质量 %.0f kg，滚阻 %.3f，Cd·A %.2f m²，电池 %.1f kWh，回收 η %.2f"
                 % (spec.mass, spec.c_rr, spec.cd * spec.area,
                    spec.battery_kwh, spec.regen_eff))
    lines.append(u"能量：驱动 %.1f kJ，制动 %.1f kJ，回收 %.1f kJ，净耗 %.1f kJ"
                 % (result["drive_j"] / 1000.0, result["brake_j"] / 1000.0,
                    result["regen_j"] / 1000.0, result["net_j"] / 1000.0))
    lines.append(u"分解：滚阻 %.1f kJ（%.1f%%），风阻 %.1f kJ（%.1f%%），"
                 u"制动 %.1f kJ（%.1f%%），动能 %.1f kJ（%.1f%%）"
                 % (result["roll_j"] / 1000.0, share(result, "roll_j") * 100.0,
                    result["aero_j"] / 1000.0, share(result, "aero_j") * 100.0,
                    result["brake_j"] / 1000.0, share(result, "brake_j") * 100.0,
                    result["ke_j"] / 1000.0, share(result, "ke_j") * 100.0))
    lines.append(u"续航：百公里 %.2f kWh，满电 %.1f km"
                 % (per_100km(result), range_km(spec, result)))
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (len([a for a in asserts if a["passed"]]),
                    len([a for a in asserts if not a["passed"]]), len(asserts)))
    for a in asserts:
        lines.append(u"  [%s] %s  %s" % (u"PASS" if a["passed"] else u"FAIL",
                                         a["type"], a["detail"]))
    ok = all(a["passed"] for a in asserts)
    lines.append(u"结论：%s" % (u"PASSED" if ok else u"FAILED"))
    return "\n".join(lines)


DEFAULT_CYCLE = [
    (5.0, 0.0, 30.0),
    (10.0, 30.0, 30.0),
    (6.0, 30.0, 50.0),
    (15.0, 50.0, 50.0),
    (8.0, 50.0, 0.0),
    (6.0, 0.0, 0.0),
    (6.0, 0.0, 40.0),
    (4.0, 40.0, 40.0),
]


class EnergyParams(object):
    """把 /xxx/yyy 形式的参数名收成对象，缺省时用内置默认值。"""

    def __init__(self, **kwargs):
        self.vehicle = VehicleSpec(
            mass=kwargs.get("vehicle/mass", 1800.0),
            c_rr=kwargs.get("vehicle/c_rr", 0.012),
            cd=kwargs.get("vehicle/cd", 0.31),
            area=kwargs.get("vehicle/area", 2.2),
            battery_kwh=kwargs.get("vehicle/battery_kwh", 60.0),
            regen_eff=kwargs.get("vehicle/regen_eff", 0.6),
        )
        self.breakdown_tolerance = kwargs.get("check/breakdown_tolerance", 0.02)
        self.regen_limit = kwargs.get("check/regen_limit", 1.0)
        self.range_tolerance = kwargs.get("check/range_tolerance", 0.02)
        self.dt = kwargs.get("cycle/dt", 0.05)
        self.cycle = kwargs.get("cycle/segments", DEFAULT_CYCLE)
