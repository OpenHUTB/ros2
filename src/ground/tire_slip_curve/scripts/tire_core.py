#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""轮胎滑移率-附着曲线核算（纯标准库，不依赖 ROS / CARLA / numpy）。

模型：以归一化滑移率 ŝ = s / s_peak 表示的单调升-单调降附着曲线

    μ(ŝ) = μ_peak * 2·ŝ / (1 + ŝ²)

    ŝ = 0   → μ = 0            （自由滚动，无纵向力）
    ŝ = 1   → μ = μ_peak       （峰值附着，唯一极大）
    ŝ > 1   → μ 单调下降       （过度滑移，附着衰减）
    ŝ = 1/s_peak 时 s = 1.0    （车轮锁死）

这条曲线的好处是：峰值位置、峰值大小、初段斜率全都有闭式解，能被断言
精确校验，不需要拟合。真实轮胎（魔术公式）在 0~0.4 的滑移区间形状与此
接近，用它讲清「为什么 ABS 要把滑移率控在峰值附近」已经足够。

Python 2.7 / 3.x 双兼容。
"""
from __future__ import division, print_function, unicode_literals

import io
import json
import math

__all__ = [
    "TireError", "TireSpec", "CurvePoint", "Curve", "build_curve",
    "key_points", "check_curve", "all_passed", "render_curve",
    "format_report", "TireParams", "PARAM_KEYS", "PARAM_DEFAULTS",
    "ASSERT_NAMES", "to_message_text",
]

# ── 默认参数 ─────────────────────────────────────────────────────────────

DEFAULT_PEAK_SLIP = 0.12
DEFAULT_PEAK_FRICTION = 0.90
DEFAULT_ABS_BAND = 0.20          # 目标带 = peak_slip * (1 ± band)
DEFAULT_SURFACE = u"dry_asphalt"
DEFAULT_SAMPLES = 51             # 曲线采样点数（s 从 0 到 1）
DEFAULT_LOCK_FRICTION_RATIO = 0.85   # 锁死附着至少降到峰值的这个比例以下
DEFAULT_LINEAR_TOLERANCE = 0.02      # 初段线性度允许的相对误差
DEFAULT_PEAK_TOLERANCE = 0.01
DEFAULT_PUBLISH_HZ = 10.0
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 12

SURFACES = (u"dry_asphalt", u"wet_asphalt", u"snow", u"ice")

ASSERT_NAMES = (u"峰值唯一", u"锁死衰减", u"初段线性", u"目标带覆盖峰值")

PARAM_DEFAULTS = {
    "tire/peak_slip": DEFAULT_PEAK_SLIP,
    "tire/peak_friction": DEFAULT_PEAK_FRICTION,
    "tire/abs_band": DEFAULT_ABS_BAND,
    "road/surface": DEFAULT_SURFACE,
    "curve/samples": DEFAULT_SAMPLES,
    "check/lock_friction_ratio": DEFAULT_LOCK_FRICTION_RATIO,
    "check/linear_tolerance": DEFAULT_LINEAR_TOLERANCE,
    "check/peak_tolerance": DEFAULT_PEAK_TOLERANCE,
    "rate/publish_hz": DEFAULT_PUBLISH_HZ,
    "topic/report": u"/tire/report",
    "topic/curve": u"/tire/curve",
    "topic/peak_slip": u"/tire/peak_slip",
}

PARAM_KEYS = tuple(sorted(PARAM_DEFAULTS))


class TireError(ValueError):
    """参数不合法。"""


# ── 小工具 ───────────────────────────────────────────────────────────────

def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise TireError(u"缺少参数：%s" % name)
        return float(default)
    if isinstance(value, bool):
        raise TireError(u"参数 %s 不能是布尔值" % name)
    try:
        out = float(value)
    except (TypeError, ValueError):
        raise TireError(u"参数 %s 不是数字：%r" % (name, value))
    if math.isnan(out) or math.isinf(out):
        raise TireError(u"参数 %s 不是有限数：%r" % (name, value))
    return out


def _as_int(value, name, default=None):
    out = _as_float(value, name, default)
    if abs(out - round(out)) > 1e-9:
        raise TireError(u"参数 %s 必须是整数：%r" % (name, value))
    return int(round(out))


def _positive(value, name):
    out = _as_float(value, name)
    if out <= 0.0:
        raise TireError(u"参数 %s 必须为正：%r" % (name, value))
    return out


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def to_message_text(text):
    """把文本转成 rospy 能安全发布的形式（py2 → bytes，py3 → str）。"""
    try:
        unicode  # noqa: F821  (Python 2)
    except NameError:
        return text
    if isinstance(text, unicode):  # noqa: F821
        return text.encode("utf-8")
    return text


# ── 数据模型 ─────────────────────────────────────────────────────────────

class TireSpec(object):
    """一条轮胎-路面组合的附着曲线参数。"""

    __slots__ = ("peak_slip", "peak_friction", "abs_band", "surface")

    def __init__(self, peak_slip=DEFAULT_PEAK_SLIP,
                 peak_friction=DEFAULT_PEAK_FRICTION,
                 abs_band=DEFAULT_ABS_BAND, surface=DEFAULT_SURFACE):
        self.peak_slip = _positive(peak_slip, "peak_slip")
        if self.peak_slip > 1.0:
            raise TireError(u"峰值滑移率必须 <= 1.0：%r" % peak_slip)
        self.peak_friction = _positive(peak_friction, "peak_friction")
        band = _as_float(abs_band, "abs_band")
        if band < 0.0 or band >= 1.0:
            raise TireError(u"ABS 目标带宽比例必须在 [0, 1)：%r" % abs_band)
        self.abs_band = band
        name = surface if surface is not None else DEFAULT_SURFACE
        if name not in SURFACES:
            raise TireError(u"未知路面：%r（可选 %s）" % (surface, ", ".join(SURFACES)))
        self.surface = name

    # 曲线与派生量的闭式解 ------------------------------------------------

    def friction(self, slip):
        """给定滑移率求附着系数。"""
        slip = _as_float(slip, "slip")
        if slip < 0.0:
            slip = -slip
        hat = slip / self.peak_slip
        return self.peak_friction * 2.0 * hat / (1.0 + hat * hat)

    def slope_at_zero(self):
        """初段刚度 dμ/ds|₀ = 2·μ_peak / s_peak（闭式解）。"""
        return 2.0 * self.peak_friction / self.peak_slip

    def lock_slip(self):
        """车轮完全锁死时的滑移率（本模型约定为 1.0）。"""
        return 1.0

    def lock_friction(self):
        return self.friction(self.lock_slip())

    def abs_band_limits(self):
        """ABS 目标带 [low, high]，并夹到 (0, 1]。"""
        low = self.peak_slip * (1.0 - self.abs_band)
        high = self.peak_slip * (1.0 + self.abs_band)
        return _clamp(low, 1e-6, 1.0), _clamp(high, 1e-6, 1.0)

    def as_tuple(self):
        return (self.peak_slip, self.peak_friction, self.abs_band, self.surface)


class CurvePoint(object):
    """曲线上的一点。"""

    __slots__ = ("slip", "friction", "ratio")

    def __init__(self, slip, friction, ratio):
        self.slip = slip
        self.friction = friction
        self.ratio = ratio          # friction / peak_friction

    def as_row(self):
        return (round(self.slip, 4), round(self.friction, 4), round(self.ratio, 4))


class Curve(object):
    """一条完整的附着曲线。"""

    def __init__(self, points, spec, name=""):
        self.points = list(points)
        self.spec = spec
        self.name = name or ("%s(peak_slip=%.3f)" % (spec.surface, spec.peak_slip))

    # 基本属性 ----------------------------------------------------------

    @property
    def count(self):
        return len(self.points)

    def is_empty(self):
        return not self.points

    def first(self):
        return self.points[0] if self.points else None

    def last(self):
        return self.points[-1] if self.points else None

    def at(self, slip):
        """取最接近给定滑移率的采样点。"""
        if not self.points:
            return None
        return min(self.points, key=lambda p: abs(p.slip - slip))

    def max_point(self):
        """曲线上附着系数最大的点。"""
        if not self.points:
            return None
        return max(self.points, key=lambda p: p.friction)

    def monotonic_up_to(self, slip):
        """[0, slip] 区间内是否严格单调递增。"""
        prev = None
        for point in self.points:
            if point.slip > slip + 1e-12:
                break
            if prev is not None and point.friction <= prev:
                return False
            prev = point.friction
        return True

    def ratio_at_lock(self):
        last = self.last()
        if last is None or last.ratio is None:
            return 0.0
        return last.ratio

    def as_rows(self):
        return [p.as_row() for p in self.points]


def build_curve(spec=None, samples=DEFAULT_SAMPLES):
    """按 s = 0 … 1 均匀采样生成曲线。"""
    spec = spec or TireSpec()
    count = _as_int(samples, "samples", DEFAULT_SAMPLES)
    if count < 5:
        raise TireError(u"采样点数至少 5：%r" % samples)
    points = []
    for index in range(count):
        slip = index / (count - 1.0)
        mu = spec.friction(slip)
        points.append(CurvePoint(slip, mu, mu / spec.peak_friction))
    return Curve(points, spec)


def key_points(spec=None):
    """返回报告里要用的关键量。"""
    spec = spec or TireSpec()
    low, high = spec.abs_band_limits()
    return {
        "surface": spec.surface,
        "peak_slip": spec.peak_slip,
        "peak_friction": spec.peak_friction,
        "lock_slip": spec.lock_slip(),
        "lock_friction": spec.lock_friction(),
        "lock_ratio": spec.lock_friction() / spec.peak_friction,
        "slope_at_zero": spec.slope_at_zero(),
        "abs_band": spec.abs_band,
        "abs_low": low,
        "abs_high": high,
        "abs_peak_friction": spec.friction(spec.peak_slip),
    }


# ── 四条断言 ─────────────────────────────────────────────────────────────

def check_curve(curve, spec=None, lock_friction_ratio=DEFAULT_LOCK_FRICTION_RATIO,
                linear_tolerance=DEFAULT_LINEAR_TOLERANCE,
                peak_tolerance=DEFAULT_PEAK_TOLERANCE):
    """返回四条断言，结构为 {type, passed, detail}。"""
    curve = curve or build_curve(spec)
    spec = spec or curve.spec
    lock_limit = _as_float(lock_friction_ratio, "lock_friction_ratio")
    lin_tol = _as_float(linear_tolerance, "linear_tolerance")
    peak_tol = _as_float(peak_tolerance, "peak_tolerance")

    results = []

    # 1) 峰值唯一：采样点里的极大值只有一个，且落在峰值滑移附近
    peak = curve.max_point()
    ups = [i for i in range(1, curve.count - 1)
           if curve.points[i].friction > curve.points[i - 1].friction
           and curve.points[i].friction > curve.points[i + 1].friction]
    peak_ok = (peak is not None
               and abs(peak.friction - spec.peak_friction) <= peak_tol
               and abs(peak.slip - spec.peak_slip) <= max(peak_tol, 2.0 / curve.count)
               and len(ups) <= 1)
    results.append({
        "type": ASSERT_NAMES[0],
        "passed": bool(peak_ok),
        "detail": u"峰值 %.4f @ s=%.4f（期望 %.4f @ %.4f），局部极大 %d 个" % (
            peak.friction if peak else 0.0, peak.slip if peak else 0.0,
            spec.peak_friction, spec.peak_slip, len(ups)),
    })

    # 2) 锁死衰减：s=1 的附着要明显低于峰值
    lock = spec.lock_friction()
    ratio = lock / spec.peak_friction
    results.append({
        "type": ASSERT_NAMES[1],
        "passed": bool(ratio <= lock_limit + 1e-12),
        "detail": u"锁死附着 %.4f，占峰值 %.1f%%（要求 <= %.1f%%）" % (
            lock, ratio * 100.0, lock_limit * 100.0),
    })

    # 3) 初段线性：s → 0 时 μ ≈ slope·s
    worst = 0.0
    worst_s = 0.0
    probe = spec.peak_slip * 0.1
    for k in range(1, 6):
        slip = probe * k / 5.0
        if slip <= 0.0:
            continue
        approx = spec.slope_at_zero() * slip
        actual = spec.friction(slip)
        rel = abs(actual - approx) / max(approx, 1e-9)
        if rel > worst:
            worst, worst_s = rel, slip
    results.append({
        "type": ASSERT_NAMES[2],
        "passed": bool(worst <= lin_tol + 1e-12),
        "detail": u"初段刚度 %.3f，最大相对误差 %.2f%% @ s=%.4f（要求 <= %.2f%%）" % (
            spec.slope_at_zero(), worst * 100.0, worst_s, lin_tol * 100.0),
    })

    # 4) 目标带覆盖峰值：ABS 目标区间必须包住峰值滑移
    low, high = spec.abs_band_limits()
    covered = low <= spec.peak_slip <= high
    results.append({
        "type": ASSERT_NAMES[3],
        "passed": bool(covered),
        "detail": u"目标带 [%.4f, %.4f] 覆盖峰值滑移 %.4f" % (low, high, spec.peak_slip),
    })

    return results


def all_passed(results):
    return all(item["passed"] for item in results)


# ── 字符图 ───────────────────────────────────────────────────────────────

def render_curve(curve, width=DEFAULT_GRID_WIDTH, height=DEFAULT_GRID_HEIGHT):
    """把 μ(s) 画成字符图：横轴滑移率 0→1，纵轴 0→peak。"""
    width = _as_int(width, "width", DEFAULT_GRID_WIDTH)
    height = _as_int(height, "height", DEFAULT_GRID_HEIGHT)
    if width < 20 or height < 5:
        raise TireError(u"网格太小：%dx%d" % (width, height))
    if curve.is_empty():
        return []

    top = curve.spec.peak_friction * 1.05
    grid = [[" "] * width for _ in range(height)]
    # 目标带底纹
    low, high = curve.spec.abs_band_limits()
    for x in range(width):
        slip = x / (width - 1.0)
        if low <= slip <= high:
            grid[height - 1][x] = "."
    # 曲线
    prev_row = None
    for x in range(width):
        slip = x / (width - 1.0)
        mu = curve.spec.friction(slip)
        row = int(round((1.0 - mu / top) * (height - 1)))
        row = int(_clamp(row, 0, height - 1))
        grid[row][x] = "*"
        if prev_row is not None and abs(row - prev_row) > 1:
            step = 1 if row > prev_row else -1
            for r in range(prev_row + step, row, step):
                grid[r][x] = "|"
        prev_row = row

    lines = ["".join(row) for row in grid]
    lines.append("0" + " " * (width - 12) + u"滑移率 → 1.0")
    return lines


# ── 报告 ─────────────────────────────────────────────────────────────────

def format_report(curve, results, header=""):
    """生成文本报告。"""
    curve = curve or build_curve()
    spec = curve.spec
    keys = key_points(spec)
    passed = sum(1 for item in results if item["passed"])
    lines = []
    if header:
        lines.append(header)
    lines.append(u"路面：%s，峰值滑移率 %.3f，峰值附着 %.3f" % (
        keys["surface"], keys["peak_slip"], keys["peak_friction"]))
    lines.append(u"锁死点：s=%.2f，附着 %.4f（占峰值 %.1f%%）" % (
        keys["lock_slip"], keys["lock_friction"], keys["lock_ratio"] * 100.0))
    lines.append(u"初段刚度：%.3f 1/s，ABS 目标带 [%.4f, %.4f]（带宽 %.0f%%）" % (
        keys["slope_at_zero"], keys["abs_low"], keys["abs_high"],
        keys["abs_band"] * 100.0))
    lines.append(u"采样：%d 点（s 从 0 到 1）" % curve.count)
    lines.append(u"-" * 60)
    lines.append(u"附着曲线（纵轴 0 ~ %.2f）：" % (spec.peak_friction * 1.05))
    for line in render_curve(curve):
        lines.append(line)
    lines.append(u"-" * 60)
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）" % (
        passed, len(results) - passed, len(results)))
    for item in results:
        lines.append(u"  [%s] %s  %s" % (
            "PASS" if item["passed"] else "FAIL", item["type"], item["detail"]))
    lines.append(u"结论：%s" % (u"PASSED" if passed == len(results) else u"FAILED"))
    return "\n".join(lines)


# ── 参数读取 ─────────────────────────────────────────────────────────────

class TireParams(object):
    """从 ROS 参数服务器读到的配置。"""

    def __init__(self, **kwargs):
        for name in PARAM_KEYS:
            attr = "params_" + name.replace("/", "_")
            setattr(self, attr, kwargs.get(name, PARAM_DEFAULTS[name]))
        self.peak_slip = _positive(self.params_tire_peak_slip, "tire/peak_slip")
        self.peak_friction = _positive(self.params_tire_peak_friction, "tire/peak_friction")
        self.abs_band = _as_float(self.params_tire_abs_band, "tire/abs_band", DEFAULT_ABS_BAND)
        surface = self.params_road_surface
        if isinstance(surface, bytes):
            surface = surface.decode("utf-8")
        self.surface = surface if surface in SURFACES else DEFAULT_SURFACE
        self.samples = _as_int(self.params_curve_samples, "curve/samples", DEFAULT_SAMPLES)
        self.lock_friction_ratio = _as_float(
            self.params_check_lock_friction_ratio, "check/lock_friction_ratio",
            DEFAULT_LOCK_FRICTION_RATIO)
        self.linear_tolerance = _as_float(
            self.params_check_linear_tolerance, "check/linear_tolerance",
            DEFAULT_LINEAR_TOLERANCE)
        self.peak_tolerance = _as_float(
            self.params_check_peak_tolerance, "check/peak_tolerance", DEFAULT_PEAK_TOLERANCE)
        self.publish_hz = _as_float(self.params_rate_publish_hz, "rate/publish_hz",
                                    DEFAULT_PUBLISH_HZ)
        self.topic_report = self.params_topic_report
        self.topic_curve = self.params_topic_curve
        self.topic_peak_slip = self.params_topic_peak_slip

    @classmethod
    def from_params(cls, get_param):
        values = {}
        for name in PARAM_KEYS:
            values[name] = get_param(name, PARAM_DEFAULTS[name])
        return cls(**values)

    def spec(self):
        return TireSpec(self.peak_slip, self.peak_friction, self.abs_band, self.surface)

    def as_dict(self):
        return {
            "tire/peak_slip": self.peak_slip,
            "tire/peak_friction": self.peak_friction,
            "tire/abs_band": self.abs_band,
            "road/surface": self.surface,
            "curve/samples": self.samples,
            "check/lock_friction_ratio": self.lock_friction_ratio,
            "check/linear_tolerance": self.linear_tolerance,
            "check/peak_tolerance": self.peak_tolerance,
            "rate/publish_hz": self.publish_hz,
        }
