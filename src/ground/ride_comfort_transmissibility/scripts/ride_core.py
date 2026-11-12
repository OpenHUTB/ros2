#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""乘坐舒适性传递率核算（纯标准库，不依赖 ROS / CARLA / numpy）。

问题：车在某个频率的路面激励下，座椅/车身会被放大多少倍？
共振峰在哪里、有多高？按 ISO 2631 加权后属于哪个舒适度等级？

模型（单自由度基础激励，位移传递率）：

    r    = f / f_n                          频率比
    T(r) = sqrt((1 + (2ζr)²) /
                ((1 − r²)² + (2ζr)²))       传递率（放大倍数）

三条闭式结论，正好用来写断言：

    峰值位置   r_p   = sqrt(1 − 2ζ²)        （ζ ≥ 1/√2 时没有峰，T ≡ 1）
    峰值大小   T_p   = 1 / (2ζ·sqrt(1 − ζ²))
    高频衰减   T(r) → 0（r 越大越快趋近，r=10 时约 1/r² 量级）

ISO 2631-1 的舒适度等级按加权加速度 a_w（m/s²，RMS）划分：

    < 0.315          舒适
    0.315 ~ 0.63     略有不适
    0.63  ~ 1.0      不适
    1.0   ~ 1.6      很不适
    1.6   ~ 2.5      极不适
    > 2.5            无法忍受
"""

from __future__ import division, print_function, unicode_literals

import math

ASSERT_NAMES = (u"共振位置", u"峰值倍率", u"高频衰减", u"输入线性")

# (上限, 等级名)，从低到高排列
COMFORT_SCALE = (
    (0.315, u"舒适"),
    (0.63, u"略有不适"),
    (1.0, u"不适"),
    (1.6, u"很不适"),
    (2.5, u"极不适"),
)


class RideError(ValueError):
    """参数非法。"""


def _positive(name, value):
    if value is None or value <= 0:
        raise RideError(u"%s 必须大于 0，收到 %r" % (name, value))
    return value


class RideSpec(object):
    """悬架与激励参数。"""

    def __init__(self, fn_hz=1.2, damping=0.25, input_accel=0.5,
                 road_freq_hz=6.0):
        self.fn_hz = _positive(u"固有频率", fn_hz)
        # 上限取 1/sqrt(2)≈0.707：超过它传递率曲线不再有经典共振峰，
        # 闭式峰位 sqrt(1-2ζ²) 也失去意义，索性判为参数非法。
        if damping <= 0 or damping >= 1.0 / math.sqrt(2.0):
            raise RideError(u"阻尼比必须在 (0, 1/sqrt(2)) 开区间，收到 %r"
                            % (damping,))
        self.damping = damping
        self.input_accel = _positive(u"输入加速度", input_accel)
        self.road_freq_hz = _positive(u"路面激励频率", road_freq_hz)

    # ---- 传递率三件套 ----
    def ratio(self, freq_hz):
        return freq_hz / self.fn_hz

    def transmissibility(self, freq_hz):
        r = self.ratio(freq_hz)
        zeta = self.damping
        num = 1.0 + (2.0 * zeta * r) ** 2
        den = (1.0 - r * r) ** 2 + (2.0 * zeta * r) ** 2
        return math.sqrt(num / den)

    def transmissibility_at_ratio(self, r):
        return self.transmissibility(r * self.fn_hz)

    def peak_ratio(self):
        return math.sqrt(1.0 - 2.0 * self.damping ** 2)

    def peak_value(self):
        """在 r_p = sqrt(1 − 2ζ²) 处取传递率，得到真实的峰值（ζ ≥ 1/√2 时为 1）。"""
        zeta = self.damping
        r2 = 1.0 - 2.0 * zeta ** 2
        w2 = 4.0 * zeta ** 2
        num = 1.0 + w2 * r2
        den = w2 * (1.0 - zeta ** 2)
        return math.sqrt(num / den)

    def weighted_accel(self, freq_hz):
        return self.transmissibility(freq_hz) * self.input_accel

    def __repr__(self):
        return ("RideSpec(fn_hz=%.3f, damping=%.3f, input_accel=%.3f)"
                % (self.fn_hz, self.damping, self.input_accel))


def comfort_grade(aw):
    """把加权加速度（m/s²）换成 ISO 2631 的等级名。"""
    if aw < 0:
        raise RideError(u"加权加速度不能为负，收到 %r" % (aw,))
    for limit, name in COMFORT_SCALE:
        if aw < limit:
            return name
    return u"无法忍受"


def scan(spec, ratios):
    """在给定频率比上取表：[(r, 频率, 传递率, 加权加速度), ...]。"""
    rows = []
    for r in ratios:
        if r <= 0:
            raise RideError(u"频率比必须大于 0，收到 %r" % (r,))
        freq = r * spec.fn_hz
        t = spec.transmissibility_at_ratio(r)
        rows.append((r, freq, t, t * spec.input_accel))
    return rows


DEFAULT_RATIOS = (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0, 10.0)


def find_peak(spec, low=0.05, high=4.0, steps=400):
    """在频率比区间上扫描，返回 (峰值频率比, 峰值传递率)。"""
    if low <= 0 or high <= low or steps < 2:
        raise RideError(u"扫描区间非法：low=%r high=%r steps=%r"
                        % (low, high, steps))
    best_r = low
    best_t = spec.transmissibility_at_ratio(low)
    for i in range(steps + 1):
        r = low + (high - low) * i / steps
        t = spec.transmissibility_at_ratio(r)
        if t > best_t:
            best_r, best_t = r, t
    return best_r, best_t


def check_ride(spec, pos_tolerance=0.02, peak_tolerance=0.05,
               high_freq_limit=0.1, linear_tolerance=0.02):
    """四条断言：共振位置、峰值倍率、高频衰减、输入线性。"""
    asserts = []

    measured_r, measured_t = find_peak(spec)
    theoretical_r = spec.peak_ratio()
    pos_error = abs(measured_r - theoretical_r)
    pos_detail = (u"扫描峰位 r=%.4f，理论 sqrt(1-2ζ²)=%.4f（偏差 %.4f，容差 %.3f）"
                  % (measured_r, theoretical_r, pos_error, pos_tolerance))
    pos_ok = pos_error <= pos_tolerance + 1e-12
    asserts.append({"type": ASSERT_NAMES[0], "passed": bool(pos_ok),
                    "detail": pos_detail})

    theoretical_t = spec.peak_value()
    rel = abs(measured_t - theoretical_t) / max(theoretical_t, 1e-12)
    asserts.append({
        "type": ASSERT_NAMES[1],
        "passed": bool(rel <= peak_tolerance + 1e-12),
        "detail": (u"扫描峰值 %.4f，理论峰值 %.4f（相对偏差 %.4f，容差 %.2f）"
                   % (measured_t, theoretical_t, rel, peak_tolerance)),
    })

    far = spec.transmissibility_at_ratio(10.0)
    asserts.append({
        "type": ASSERT_NAMES[2],
        "passed": bool(far <= high_freq_limit + 1e-12),
        "detail": (u"r=10 时传递率 %.6f，上限 %.3f（高频必须衰减）"
                   % (far, high_freq_limit)),
    })

    doubled = RideSpec(fn_hz=spec.fn_hz, damping=spec.damping,
                       input_accel=2.0 * spec.input_accel)
    base = spec.weighted_accel(spec.road_freq_hz)
    dbl = doubled.weighted_accel(spec.road_freq_hz)
    rel_lin = abs(dbl - 2.0 * base) / max(2.0 * base, 1e-12)
    asserts.append({
        "type": ASSERT_NAMES[3],
        "passed": bool(rel_lin <= linear_tolerance + 1e-12),
        "detail": (u"输入 %.2f → 加权 %.4f；输入加倍 → 加权 %.4f（相对偏差 %.4f）"
                   % (spec.input_accel, base, dbl, rel_lin)),
    })
    return asserts


def render_transmissibility(spec, ratios=DEFAULT_RATIOS, width=40):
    """按传递率画横向条形图（以峰值为满条）。"""
    peak = spec.peak_value()
    peak = peak if peak > 0 else 1.0
    lines = [u"传递率曲线（峰值 = 满条）", "-" * (width + 16)]
    for r, freq, t, aw in scan(spec, ratios):
        cells = int(round(t / peak * width))
        mark = u" <峰值" if abs(r - spec.peak_ratio()) < 0.13 else u""
        lines.append(u"r=%4.2f f=%5.2fHz |%s| T=%.3f%s"
                     % (r, freq, "#" * cells, t, mark))
    lines.append("-" * (width + 16))
    return "\n".join(lines)


def format_report(spec, asserts, ratios=DEFAULT_RATIOS):
    """把参数、扫描表、断言与结论排成报告。"""
    lines = []
    peak_r, peak_t = find_peak(spec)
    lines.append(u"悬架：固有频率 %.3f Hz，阻尼比 %.3f，路面激励 %.2f Hz，输入 %.2f m/s²"
                 % (spec.fn_hz, spec.damping, spec.road_freq_hz,
                    spec.input_accel))
    lines.append(u"理论：峰位 r=%.4f，峰值 %.4f；ζ=%.3f"
                 % (spec.peak_ratio(), spec.peak_value(), spec.damping))
    lines.append(u"扫描：峰位 r=%.4f，峰值 %.4f" % (peak_r, peak_t))
    aw = spec.weighted_accel(spec.road_freq_hz)
    lines.append(u"舒适性：加权加速度 %.3f m/s² → 等级「%s」"
                 % (aw, comfort_grade(aw)))
    lines.append(u"频率比  频率(Hz)  传递率  加权(m/s²)")
    for r, freq, t, a in scan(spec, ratios):
        lines.append(u"%6.2f  %8.3f  %6.3f  %8.3f" % (r, freq, t, a))
    lines.append(u"断言：%d 通过 / %d 失败（共 %d 条）"
                 % (len([a for a in asserts if a["passed"]]),
                    len([a for a in asserts if not a["passed"]]), len(asserts)))
    for a in asserts:
        lines.append(u"  [%s] %s  %s" % (u"PASS" if a["passed"] else u"FAIL",
                                         a["type"], a["detail"]))
    ok = all(a["passed"] for a in asserts)
    lines.append(u"结论：%s" % (u"PASSED" if ok else u"FAILED"))
    return "\n".join(lines)


class RideParams(object):
    """把 /xxx/yyy 形式的参数名收成对象，缺省时用内置默认值。"""

    def __init__(self, **kwargs):
        self.spec = RideSpec(
            fn_hz=kwargs.get("suspension/fn_hz", 1.2),
            damping=kwargs.get("suspension/damping", 0.25),
            input_accel=kwargs.get("road/input_accel", 0.5),
            road_freq_hz=kwargs.get("road/freq_hz", 6.0),
        )
        self.pos_tolerance = kwargs.get("check/pos_tolerance", 0.02)
        self.peak_tolerance = kwargs.get("check/peak_tolerance", 0.05)
        self.high_freq_limit = kwargs.get("check/high_freq_limit", 0.1)
        self.linear_tolerance = kwargs.get("check/linear_tolerance", 0.02)
        self.scan_low = kwargs.get("scan/low", 0.05)
        self.scan_high = kwargs.get("scan/high", 4.0)
        self.scan_steps = int(kwargs.get("scan/steps", 400))
