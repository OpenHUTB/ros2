#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""横向载荷转移与侧翻裕度核算（纯标准库，不依赖 ROS / CARLA / numpy）。

问题：一辆车以速度 v 过半径 R 的弯，会不会翻？

准静态模型只有三个量：

    静态稳定因子   SSF = track / (2 · h_cg)
    侧翻临界加速度 a_crit = SSF · g
    轮距侧载荷转移 ΔF_z = m · a_y · h_cg / track

于是：

    内侧轮垂向力 F_inner = m·g/2 − ΔF_z
    外侧轮垂向力 F_outer = m·g/2 + ΔF_z
    抬轮（内侧轮离地）恰好发生在 ΔF_z = m·g/2，
    也就是 a_y = g · track / (2·h_cg) = a_crit —— 两条定义自洽。

给定弯道半径还能反推极限车速 v_max = sqrt(a_crit · R)。
这套关系全部是代数式，可以被断言精确校验，不需要仿真器。

Python 2.7 / 3.x 双兼容。
"""
from __future__ import division, print_function, unicode_literals

import io
import math

__all__ = [
    "RolloverError", "VehicleSpec", "Cornering", "check_cornering", "all_passed",
    "speed_table", "render_loads", "format_report", "RolloverParams",
    "PARAM_KEYS", "PARAM_DEFAULTS", "ASSERT_NAMES", "SPEED_TABLE_KMH",
    "to_message_text",
]

# ── 默认参数 ─────────────────────────────────────────────────────────────

DEFAULT_TRACK = 1.54            # 轮距（m）
DEFAULT_CG_HEIGHT = 0.55        # 质心高度（m）
DEFAULT_MASS = 1500.0           # 整车质量（kg）
DEFAULT_CURVE_RADIUS = 30.0     # 弯道半径（m）
DEFAULT_SPEED_KMH = 60.0        # 过弯车速（km/h）
DEFAULT_ROLLOVER_MARGIN = 0.70  # 允许的 a_y / a_crit 上限
DEFAULT_TRANSFER_TOLERANCE = 0.02
DEFAULT_GRAVITY = 9.81
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 12

SPEED_TABLE_KMH = (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0)

ASSERT_NAMES = (u"侧翻裕度", u"内侧轮载荷", u"极限车速", u"载荷转移")

PARAM_DEFAULTS = {
    "vehicle/track": DEFAULT_TRACK,
    "vehicle/cg_height": DEFAULT_CG_HEIGHT,
    "vehicle/mass": DEFAULT_MASS,
    "vehicle/gravity": DEFAULT_GRAVITY,
    "corner/radius": DEFAULT_CURVE_RADIUS,
    "corner/speed_kmh": DEFAULT_SPEED_KMH,
    "check/rollover_margin": DEFAULT_ROLLOVER_MARGIN,
    "check/transfer_tolerance": DEFAULT_TRANSFER_TOLERANCE,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
    "topic/report": u"/rollover/report",
    "topic/loads": u"/rollover/loads",
    "topic/critical_speed": u"/rollover/critical_speed",
}

PARAM_KEYS = tuple(sorted(PARAM_DEFAULTS))


class RolloverError(ValueError):
    """参数不合法。"""


# ── 小工具 ───────────────────────────────────────────────────────────────

def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise RolloverError(u"缺少参数：%s" % name)
        return float(default)
    if isinstance(value, bool):
        raise RolloverError(u"参数 %s 不能是布尔值" % name)
    try:
        out = float(value)
    except (TypeError, ValueError):
        raise RolloverError(u"参数 %s 不是数字：%r" % (name, value))
    if math.isnan(out) or math.isinf(out):
        raise RolloverError(u"参数 %s 不是有限数：%r" % (name, value))
    return out


def _positive(value, name):
    out = _as_float(value, name)
    if out <= 0.0:
        raise RolloverError(u"参数 %s 必须为正：%r" % (name, value))
    return out


def _non_negative(value, name):
    out = _as_float(value, name)
    if out < 0.0:
        raise RolloverError(u"参数 %s 不能为负：%r" % (name, value))
    return out


def to_message_text(text):
    try:
        unicode  # noqa: F821
    except NameError:
        return text
    if isinstance(text, unicode):  # noqa: F821
        return text.encode("utf-8")
    return text


# ── 数据模型 ─────────────────────────────────────────────────────────────

class VehicleSpec(object):
    """车辆的准静态侧倾相关几何与质量参数。"""

    __slots__ = ("track", "cg_height", "mass", "gravity")

    def __init__(self, track=DEFAULT_TRACK, cg_height=DEFAULT_CG_HEIGHT,
                 mass=DEFAULT_MASS, gravity=DEFAULT_GRAVITY):
        self.track = _positive(track, "track")
        self.cg_height = _positive(cg_height, "cg_height")
        self.mass = _positive(mass, "mass")
        self.gravity = _positive(gravity, "gravity")

    # 基本量 ------------------------------------------------------------

    def ssf(self):
        """静态稳定因子 SSF = track / (2·h_cg)。"""
        return self.track / (2.0 * self.cg_height)

    def critical_accel(self):
        """侧翻临界侧向加速度（m/s²），等于 SSF·g。"""
        return self.ssf() * self.gravity

    def wheel_load_static(self):
        """单侧静态垂向力（N）。"""
        return self.mass * self.gravity / 2.0

    def transfer(self, lateral_accel):
        """横向载荷转移量 ΔF_z（N），正数表示从内侧移到外侧。"""
        ay = _as_float(lateral_accel, "lateral_accel")
        return self.mass * ay * self.cg_height / self.track

    def inner_load(self, lateral_accel):
        return self.wheel_load_static() - self.transfer(lateral_accel)

    def outer_load(self, lateral_accel):
        return self.wheel_load_static() + self.transfer(lateral_accel)

    def critical_speed(self, radius):
        """给定半径的极限过弯车速（m/s）。"""
        radius = _positive(radius, "radius")
        return math.sqrt(self.critical_accel() * radius)

    def as_tuple(self):
        return (self.track, self.cg_height, self.mass, self.gravity)


class Cornering(object):
    """一次过弯工况。"""

    __slots__ = ("spec", "radius", "speed_kmh", "speed_mps")

    def __init__(self, spec=None, radius=DEFAULT_CURVE_RADIUS,
                 speed_kmh=DEFAULT_SPEED_KMH):
        self.spec = spec or VehicleSpec()
        self.radius = _positive(radius, "radius")
        self.speed_kmh = _as_float(speed_kmh, "speed_kmh")
        if self.speed_kmh < 0.0:
            raise RolloverError(u"车速不能为负：%r" % speed_kmh)
        self.speed_mps = self.speed_kmh / 3.6

    # 派生量 ------------------------------------------------------------

    def lateral_accel(self):
        return self.speed_mps * self.speed_mps / self.radius

    def usage(self):
        """a_y / a_crit，越小越安全；>= 1 表示已抬轮。"""
        return self.lateral_accel() / self.spec.critical_accel()

    def transfer(self):
        return self.spec.transfer(self.lateral_accel())

    def inner_load(self):
        return self.spec.inner_load(self.lateral_accel())

    def outer_load(self):
        return self.spec.outer_load(self.lateral_accel())

    def transfer_ratio(self):
        """ΔF_z / F_z0，理论上恒等于 a_y / a_crit。"""
        return self.transfer() / self.spec.wheel_load_static()

    def lifts_off(self):
        return self.inner_load() <= 0.0

    def speed_limit(self):
        return self.spec.critical_speed(self.radius)

    def speed_limit_kmh(self):
        return self.speed_limit() * 3.6

    def headroom_kmh(self):
        return self.speed_limit_kmh() - self.speed_kmh

    def safe_speed(self, margin=DEFAULT_ROLLOVER_MARGIN):
        """在给定裕度下允许的最高车速（km/h）。"""
        margin = _positive(margin, "rollover_margin")
        return math.sqrt(margin * self.spec.critical_accel() * self.radius) * 3.6

    def as_row(self):
        return (round(self.speed_kmh, 2), round(self.lateral_accel(), 4),
                round(self.usage(), 4), bool(self.lifts_off()))


# ── 四条断言 ─────────────────────────────────────────────────────────────

def check_cornering(cornering, rollover_margin=DEFAULT_ROLLOVER_MARGIN,
                    transfer_tolerance=DEFAULT_TRANSFER_TOLERANCE):
    """返回四条断言，结构为 {type, passed, detail}。"""
    cornering = cornering or Cornering()
    margin = _positive(rollover_margin, "rollover_margin")
    tol = _as_float(transfer_tolerance, "transfer_tolerance")
    spec = cornering.spec
    usage = cornering.usage()
    results = []

    # 1) 侧翻裕度
    results.append({
        "type": ASSERT_NAMES[0],
        "passed": bool(usage <= margin + 1e-12),
        "detail": u"a_y %.3f m/s²，a_crit %.3f m/s²，比值 %.3f（上限 %.2f）" % (
            cornering.lateral_accel(), spec.critical_accel(), usage, margin),
    })

    # 2) 内侧轮载荷
    inner = cornering.inner_load()
    results.append({
        "type": ASSERT_NAMES[1],
        "passed": bool(inner > 0.0),
        "detail": u"内侧轮 %.1f N，外侧轮 %.1f N（静态 %.1f N）" % (
            inner, cornering.outer_load(), spec.wheel_load_static()),
    })

    # 3) 极限车速
    limit = cornering.speed_limit_kmh()
    results.append({
        "type": ASSERT_NAMES[2],
        "passed": bool(cornering.speed_kmh <= limit + 1e-9),
        "detail": u"当前 %.1f km/h，该半径极限 %.1f km/h（余量 %.1f km/h）" % (
            cornering.speed_kmh, limit, cornering.headroom_kmh()),
    })

    # 4) 载荷转移自洽：ΔF_z/F_z0 应等于 a_y/a_crit
    ratio = cornering.transfer_ratio()
    gap = abs(ratio - usage)
    results.append({
        "type": ASSERT_NAMES[3],
        "passed": bool(gap <= tol + 1e-12),
        "detail": u"ΔF_z/F_z0 = %.4f，a_y/a_crit = %.4f，偏差 %.2e（容差 %.2f）" % (
            ratio, usage, gap, tol),
    })

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


def speed_table(spec=None, radius=DEFAULT_CURVE_RADIUS, speeds=SPEED_TABLE_KMH):
    """同一半径下、不同车速的裕度表。"""
    spec = spec or VehicleSpec()
    rows = []
    for kmh in speeds:
        item = Cornering(spec, radius, kmh)
        rows.append(item)
    return rows


# ── 字符图 ───────────────────────────────────────────────────────────────

def render_loads(cornering, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """画两条横向载荷条：内侧（左）与外侧（右），中间是静态载荷中线。"""
    width = int(_as_float(width, "width", DEFAULT_GRID_WIDTH))
    height = int(_as_float(height, "height", DEFAULT_GRID_HEIGHT))
    if width < 20 or height < 5:
        raise RolloverError(u"网格太小：%dx%d" % (width, height))
    spec = cornering.spec
    total = spec.mass * spec.gravity
    scale = max(total, 1e-9)
    inner = cornering.inner_load()
    outer = cornering.outer_load()
    inner_cells = int(round(max(0.0, inner) / scale * (width // 2 - 2)))
    outer_cells = int(round(max(0.0, outer) / scale * (width // 2 - 2)))

    lines = []
    mid = width // 2
    for row in range(height):
        cells = [" "] * width
        cells[mid] = "|"
        # 内侧（左）从中间往左画
        if inner_cells > 0:
            for k in range(inner_cells):
                pos = mid - 1 - k
                if pos >= 0:
                    cells[pos] = "#"
        # 外侧（右）从中间往右画
        if outer_cells > 0:
            for k in range(outer_cells):
                pos = mid + 1 + k
                if pos < width:
                    cells[pos] = "="
        lines.append("".join(cells))
    lines.append(u"内侧 %5.0f N%s %.0f N 外侧" % (inner, " " * max(1, width // 2 - 16), outer))
    return lines


# ── 报告 ─────────────────────────────────────────────────────────────────

def format_report(cornering, results, header=""):
    cornering = cornering or Cornering()
    spec = cornering.spec
    passed = sum(1 for item in results if item["passed"])
    lines = []
    if header:
        lines.append(header)
    lines.append(u"车辆：轮距 %.2f m，质心高 %.2f m，质量 %.0f kg" % (
        spec.track, spec.cg_height, spec.mass))
    lines.append(u"SSF = %.4f，侧翻临界加速度 %.3f m/s²（%.2f g）" % (
        spec.ssf(), spec.critical_accel(), spec.ssf()))
    lines.append(u"工况：半径 %.1f m，车速 %.1f km/h（%.2f m/s）" % (
        cornering.radius, cornering.speed_kmh, cornering.speed_mps))
    lines.append(u"侧向加速度 %.3f m/s²，裕度比值 %.3f" % (
        cornering.lateral_accel(), cornering.usage()))
    lines.append(u"载荷转移 %.1f N（占比 %.3f），内侧 %.1f N / 外侧 %.1f N" % (
        cornering.transfer(), cornering.transfer_ratio(),
        cornering.inner_load(), cornering.outer_load()))
    lines.append(u"该半径极限车速 %.1f km/h（余量 %.1f km/h）" % (
        cornering.speed_limit_kmh(), cornering.headroom_kmh()))
    lines.append(u"-" * 60)
    lines.append(u"横向载荷（| 为静态中线，# 内侧，= 外侧）：")
    for line in render_loads(cornering):
        lines.append(line)
    lines.append(u"-" * 60)
    lines.append(u"车速扫描（半径 %.1f m）：" % cornering.radius)
    lines.append(u"  车速(km/h)  a_y(m/s²)  裕度   内侧轮(N)  是否抬轮")
    for item in speed_table(spec, cornering.radius):
        lines.append(u"  %9.1f  %9.3f  %5.3f  %9.1f  %s" % (
            item.speed_kmh, item.lateral_accel(), item.usage(),
            item.inner_load(), u"是" if item.lifts_off() else u"否"))
    lines.append(u"-" * 60)
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）" % (
        passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %s  %s" % (
            "PASS" if item["passed"] else "FAIL", item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if passed == len(results) else u"FAILED"))
    return "\n".join(lines)


# ── 参数读取 ─────────────────────────────────────────────────────────────

class RolloverParams(object):
    def __init__(self, **kwargs):
        for name in PARAM_KEYS:
            setattr(self, "params_" + name.replace("/", "_"),
                    kwargs.get(name, PARAM_DEFAULTS[name]))
        self.track = _positive(self.params_vehicle_track, "vehicle/track")
        self.cg_height = _positive(self.params_vehicle_cg_height, "vehicle/cg_height")
        self.mass = _positive(self.params_vehicle_mass, "vehicle/mass")
        self.gravity = _positive(self.params_vehicle_gravity, "vehicle/gravity")
        self.radius = _positive(self.params_corner_radius, "corner/radius")
        self.speed_kmh = _non_negative(self.params_corner_speed_kmh, "corner/speed_kmh")
        self.rollover_margin = _positive(
            self.params_check_rollover_margin, "check/rollover_margin")
        self.transfer_tolerance = _non_negative(
            self.params_check_transfer_tolerance, "check/transfer_tolerance")
        self.publish_hz = _positive(self.params_rate_publish_hz, "rate/publish_hz")
        self.topic_report = self.params_topic_report
        self.topic_loads = self.params_topic_loads
        self.topic_critical_speed = self.params_topic_critical_speed

    @classmethod
    def from_params(cls, get_param):
        values = {}
        for name in PARAM_KEYS:
            values[name] = get_param(name, PARAM_DEFAULTS[name])
        return cls(**values)

    def vehicle(self):
        return VehicleSpec(self.track, self.cg_height, self.mass, self.gravity)

    def cornering(self):
        return Cornering(self.vehicle(), self.radius, self.speed_kmh)

    def as_dict(self):
        return {
            "vehicle/track": self.track,
            "vehicle/cg_height": self.cg_height,
            "vehicle/mass": self.mass,
            "vehicle/gravity": self.gravity,
            "corner/radius": self.radius,
            "corner/speed_kmh": self.speed_kmh,
            "check/rollover_margin": self.rollover_margin,
            "check/transfer_tolerance": self.transfer_tolerance,
            "rate/publish_hz": self.publish_hz,
        }
