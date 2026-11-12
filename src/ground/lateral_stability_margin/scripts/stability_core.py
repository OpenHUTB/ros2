#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""不足转向梯度与侧向稳定裕度核算（纯标准库，不依赖 ROS / CARLA / numpy）。

问题：一辆车是「推头」还是「甩尾」？在多大车速下会失去侧向稳定？
方向盘打多少度，才能维持住某个半径的弯？

模型（线性轮胎，自行车模型）：

    轴荷      W_f = m·g·b/L            前轴静态载荷（b = 质心到后轴）
              W_r = m·g·a/L            后轴静态载荷（a = 质心到前轴）
    不足转向梯度
              K = W_f / C_f − W_r / C_r      [rad / (m/s²)]
                  K > 0 不足转向（推头，稳定）
                  K = 0 中性转向
                  K < 0 过多转向（甩尾，会失稳）
    特征车速  v_char = sqrt(L·g / K)      （K > 0 才有）
    临界车速  v_crit = sqrt(−L·g / K)     （K < 0 才有，超过就发散）
    稳态转向角
              δ = L/R + K·a_y             [rad]，a_y = v²/R
              低速时 K·a_y → 0，退化成阿克曼角 L/R
    转向灵敏度
              (a_y / δ) = v² / (L + K·v²)  分母在 v_crit 处为 0
"""

from __future__ import division, print_function, unicode_literals

import math

ASSERT_NAMES = (u"轴荷分配", u"不足转向裕度", u"稳态转向角", u"低速阿克曼")

GRAVITY = 9.81


class StabilityError(ValueError):
    """参数非法。"""


def _positive(name, value):
    if value is None or value <= 0:
        raise StabilityError(u"%s 必须大于 0，收到 %r" % (name, value))
    return value


class VehicleSpec(object):
    """整车几何与轮胎侧偏刚度。"""

    def __init__(self, mass=1500.0, wheelbase=2.7, cg_to_front=1.2,
                 front_cornering=80000.0, rear_cornering=95000.0,
                 gravity=GRAVITY):
        self.mass = _positive(u"整车质量", mass)
        self.wheelbase = _positive(u"轴距", wheelbase)
        self.cg_to_front = _positive(u"质心到前轴距离", cg_to_front)
        if self.cg_to_front >= self.wheelbase:
            raise StabilityError(u"质心到前轴距离必须小于轴距，收到 %r"
                                 % (self.cg_to_front,))
        self.front_cornering = _positive(u"前轮侧偏刚度", front_cornering)
        self.rear_cornering = _positive(u"后轮侧偏刚度", rear_cornering)
        self.gravity = _positive(u"重力加速度", gravity)

    # ---- 几何 ----
    def cg_to_rear(self):
        return self.wheelbase - self.cg_to_front

    def front_load(self):
        return self.mass * self.gravity * self.cg_to_rear() / self.wheelbase

    def rear_load(self):
        return self.mass * self.gravity * self.cg_to_front / self.wheelbase

    def total_load(self):
        return self.mass * self.gravity

    # ---- 不足转向梯度 ----
    def gradient(self):
        """不足转向梯度 K [rad/(m/s²)] = (W_f/C_f − W_r/C_r) / g。

        推导：前轮侧偏角 α_f = W_f·a_y/(g·C_f)，后轮同理，
        稳态转向角 δ = L/R + α_f − α_r。
        """
        return (self.front_load() / self.front_cornering
                - self.rear_load() / self.rear_cornering) / self.gravity

    def character(self):
        k = self.gradient()
        if abs(k) < 1e-12:
            return u"中性转向"
        return u"不足转向" if k > 0 else u"过多转向"

    def characteristic_speed(self):
        """特征车速（m/s）：转向灵敏度降到中性转向一半的车速；K ≤ 0 时不存在。"""
        k = self.gradient()
        if k <= 0:
            return None
        return math.sqrt(self.wheelbase / k)

    def critical_speed(self):
        """临界车速（m/s）：转向灵敏度发散的车速；K ≥ 0 时不存在。"""
        k = self.gradient()
        if k >= 0:
            return None
        return math.sqrt(-self.wheelbase / k)

    def __repr__(self):
        return ("VehicleSpec(mass=%.1f, wheelbase=%.2f, cg_to_front=%.2f, "
                "Cf=%.0f, Cr=%.0f)"
                % (self.mass, self.wheelbase, self.cg_to_front,
                   self.front_cornering, self.rear_cornering))


class Cornering(object):
    """给定半径与车速的一次稳态圆周。"""

    def __init__(self, spec, radius=50.0, speed_kmh=40.0):
        if spec is None:
            raise StabilityError(u"缺少整车参数")
        self.spec = spec
        self.radius = _positive(u"转弯半径", radius)
        self.speed_kmh = _positive(u"车速", speed_kmh)

    def speed(self):
        return self.speed_kmh / 3.6

    def lateral_accel(self):
        return self.speed() ** 2 / self.radius

    def steer_angle(self):
        """稳态转向角（rad）= L/R + K·a_y。"""
        return (self.spec.wheelbase / self.radius
                + self.spec.gradient() * self.lateral_accel())

    def ackermann_angle(self):
        """低速理想阿克曼角（rad）= L/R。"""
        return self.spec.wheelbase / self.radius

    def deflection(self):
        """转向角超出阿克曼角的部分（rad），来自不足/过多转向。"""
        return self.steer_angle() - self.ackermann_angle()

    def sensitivity(self):
        """转向灵敏度 a_y / δ；分母非正时返回 None（已过临界点）。"""
        den = self.spec.wheelbase + self.spec.gradient() * self.speed() ** 2
        if den <= 0:
            return None
        return self.lateral_accel() / self.steer_angle()

    def __repr__(self):
        return ("Cornering(radius=%.1f, speed_kmh=%.1f)"
                % (self.radius, self.speed_kmh))


def check_stability(cornering, understeer_min=0.0, ackermann_speed_kmh=2.0,
                    ackermann_tolerance=0.02, load_tolerance=1e-6):
    """四条断言：轴荷分配、不足转向裕度、稳态转向角、低速阿克曼。"""
    spec = cornering.spec
    asserts = []

    total = spec.total_load()
    split = spec.front_load() + spec.rear_load()
    rel = abs(split - total) / max(total, 1e-12)
    ratio_ok = abs(spec.front_load() / spec.rear_load()
                   - spec.cg_to_rear() / spec.cg_to_front) < 1e-9
    asserts.append({
        "type": ASSERT_NAMES[0],
        "passed": bool(rel <= load_tolerance and ratio_ok),
        "detail": (u"前轴 %.1f N + 后轴 %.1f N = %.1f N（整车 %.1f N，相对偏差 %.2e）"
                   % (spec.front_load(), spec.rear_load(), split, total, rel)),
    })

    k = spec.gradient()
    if k > 0:
        extra = u"特征车速 %.3f m/s（%.1f km/h）" % (
            spec.characteristic_speed(), spec.characteristic_speed() * 3.6)
    elif k < 0:
        extra = u"临界车速 %.3f m/s（%.1f km/h）" % (
            spec.critical_speed(), spec.critical_speed() * 3.6)
    else:
        extra = u"无特征/临界车速"
    asserts.append({
        "type": ASSERT_NAMES[1],
        "passed": bool(k >= understeer_min - 1e-12),
        "detail": (u"K = %.6f rad/(m/s²)（下限 %.4f），特性「%s」，%s"
                   % (k, understeer_min, spec.character(), extra)),
    })

    slow = Cornering(spec, radius=cornering.radius,
                     speed_kmh=ackermann_speed_kmh)

    # 闭环自洽：由 δ 反算 a_y，再代回 δ = L/R + K·a_y
    delta = cornering.steer_angle()
    ay_direct = cornering.lateral_accel()
    ay_back = (delta - spec.wheelbase / cornering.radius) / spec.gradient() \
        if abs(spec.gradient()) > 1e-12 else ay_direct
    rel_loop = abs(ay_back - ay_direct) / max(abs(ay_direct), 1e-12)
    asserts.append({
        "type": ASSERT_NAMES[2],
        "passed": bool(rel_loop <= 1e-6 and delta > 0),
        "detail": (u"δ = %.6f rad → 反算 a_y = %.6f m/s²，正算 %.6f m/s²"
                   u"（相对偏差 %.2e）"
                   % (delta, ay_back, ay_direct, rel_loop)),
    })

    dev = abs(slow.steer_angle() - slow.ackermann_angle())
    rel_dev = dev / max(slow.ackermann_angle(), 1e-12)
    asserts.append({
        "type": ASSERT_NAMES[3],
        "passed": bool(rel_dev <= ackermann_tolerance + 1e-12),
        "detail": (u"车速 %.1f km/h 时转向角 %.6f rad，阿克曼角 %.6f rad"
                   u"（相对偏差 %.4f，容差 %.3f）"
                   % (ackermann_speed_kmh, slow.steer_angle(),
                      slow.ackermann_angle(), rel_dev, ackermann_tolerance)),
    })
    return asserts


DEFAULT_SPEEDS_KMH = (20.0, 40.0, 60.0, 80.0, 100.0, 120.0)


def speed_table(spec, radius=50.0, speeds=DEFAULT_SPEEDS_KMH):
    """车速扫描：[(车速 km/h, a_y, δ rad, δ °, 灵敏度)]。"""
    rows = []
    for v in speeds:
        c = Cornering(spec, radius=radius, speed_kmh=v)
        rows.append((v, c.lateral_accel(), c.steer_angle(),
                     math.degrees(c.steer_angle()), c.sensitivity()))
    return rows


def render_speed_table(spec, radius=50.0, speeds=DEFAULT_SPEEDS_KMH, width=24):
    """按转向角画横向条形图（以最大角为满条）。"""
    rows = speed_table(spec, radius, speeds)
    top = max(r[2] for r in rows) if rows else 1.0
    top = top if top > 0 else 1.0
    lines = [u"转向角随车速（半径 %.1f m，满条 = 最大角）" % radius,
             "-" * (width + 34)]
    for v, ay, delta, deg, sens in rows:
        cells = int(round(delta / top * width))
        sens_txt = u"%7.3f 1/s" % sens if sens is not None else u"  发散   "
        lines.append(u"%5.0f km/h |%s| %7.4f rad %s"
                     % (v, "#" * cells, delta, sens_txt))
    lines.append("-" * (width + 34))
    return "\n".join(lines)


def format_report(spec, cornering, asserts,
                  speeds=DEFAULT_SPEEDS_KMH):
    """把参数、扫描表、断言与结论排成报告。"""
    lines = []
    lines.append(u"整车：质量 %.1f kg，轴距 %.2f m，质心到前轴 %.2f m（到后轴 %.2f m）"
                 % (spec.mass, spec.wheelbase, spec.cg_to_front,
                    spec.cg_to_rear()))
    lines.append(u"轮胎：前轮侧偏刚度 %.0f N/rad，后轮 %.0f N/rad"
                 % (spec.front_cornering, spec.rear_cornering))
    lines.append(u"轴荷：前 %.1f N，后 %.1f N（合计 %.1f N）"
                 % (spec.front_load(), spec.rear_load(), spec.total_load()))
    lines.append(u"梯度：K = %.6f rad/(m/s²)，特性「%s」"
                 % (spec.gradient(), spec.character()))
    if spec.characteristic_speed() is not None:
        lines.append(u"特征车速 %.3f m/s（%.1f km/h）"
                     % (spec.characteristic_speed(),
                        spec.characteristic_speed() * 3.6))
    if spec.critical_speed() is not None:
        lines.append(u"临界车速 %.3f m/s（%.1f km/h）"
                     % (spec.critical_speed(), spec.critical_speed() * 3.6))
    lines.append(u"工况：半径 %.1f m，车速 %.1f km/h → a_y %.3f m/s²，"
                 u"δ %.6f rad（%.2f°），阿克曼角 %.6f rad（%.2f°），"
                 u"超出 %.6f rad"
                 % (cornering.radius, cornering.speed_kmh,
                    cornering.lateral_accel(), cornering.steer_angle(),
                    math.degrees(cornering.steer_angle()),
                    cornering.ackermann_angle(),
                    math.degrees(cornering.ackermann_angle()),
                    cornering.deflection()))
    sens = cornering.sensitivity()
    lines.append(u"转向灵敏度 a_y/δ = %s"
                 % (u"%.4f 1/s" % sens if sens is not None else u"已发散（过临界）"))
    lines.append(u"车速(km/h)  a_y(m/s²)   δ(rad)   δ(°)   灵敏度(1/s)")
    for v, ay, delta, deg, sens in speed_table(spec, cornering.radius, speeds):
        lines.append(u"%9.0f  %9.3f  %8.4f %7.2f  %s"
                     % (v, ay, delta, deg,
                        u"%8.4f" % sens if sens is not None else u"   发散"))
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (len([a for a in asserts if a["passed"]]),
                    len([a for a in asserts if not a["passed"]]), len(asserts)))
    for a in asserts:
        lines.append(u"  [%s] %s  %s" % (u"PASS" if a["passed"] else u"FAIL",
                                         a["type"], a["detail"]))
    ok = all(a["passed"] for a in asserts)
    lines.append(u"结论：%s" % (u"PASSED" if ok else u"FAILED"))
    return "\n".join(lines)


class StabilityParams(object):
    """把 /xxx/yyy 形式的参数名收成对象，缺省时用内置默认值。"""

    def __init__(self, **kwargs):
        self.spec = VehicleSpec(
            mass=kwargs.get("vehicle/mass", 1500.0),
            wheelbase=kwargs.get("vehicle/wheelbase", 2.7),
            cg_to_front=kwargs.get("vehicle/cg_to_front", 1.2),
            front_cornering=kwargs.get("tire/front_cornering", 80000.0),
            rear_cornering=kwargs.get("tire/rear_cornering", 95000.0),
            gravity=kwargs.get("vehicle/gravity", GRAVITY),
        )
        self.radius = kwargs.get("scene/radius", 50.0)
        self.speed_kmh = kwargs.get("scene/speed_kmh", 40.0)
        self.understeer_min = kwargs.get("check/understeer_min", 0.0)
        self.ackermann_speed_kmh = kwargs.get("check/ackermann_speed_kmh", 2.0)
        self.ackermann_tolerance = kwargs.get("check/ackermann_tolerance", 0.02)
        self.speeds = kwargs.get("scan/speeds_kmh", DEFAULT_SPEEDS_KMH)
