"""把仿真结果画成 PNG，全部基于 render 里的索引色画布，不依赖任何绘图库。"""
from __future__ import annotations

import math

from . import render
from .sim import SimResult

BG = render.BG
TEXT = render.TEXT
MUTED = render.MUTED
GRID = render.GRID
C_ACTIVE = render.C_A      # 半主动
C_PASSIVE = render.C_B     # 被动
C_REF = render.C_C
C_EDGE = render.EDGE


class Plot:
    """一块坐标区，负责数据坐标到像素坐标的映射。"""

    def __init__(self, canvas, x0, y0, x1, y1, xmin, xmax, ymin, ymax):
        self.c = canvas
        self.x0, self.x1 = x0, x1
        self.y0, self.y1 = y0, y1
        self.xmin, self.xmax = xmin, xmax
        self.ymin, self.ymax = ymin, ymax

    def px(self, x, y):
        sx = self.x0 + (x - self.xmin) / (self.xmax - self.xmin) * (self.x1 - self.x0)
        sy = self.y1 - (y - self.ymin) / (self.ymax - self.ymin) * (self.y1 - self.y0)
        return sx, sy

    def frame(self):
        self.c.line(self.x0, self.y0, self.x1, self.y0, C_EDGE)
        self.c.line(self.x1, self.y0, self.x1, self.y1, C_EDGE)
        self.c.line(self.x1, self.y1, self.x0, self.y1, C_EDGE)
        self.c.line(self.x0, self.y1, self.x0, self.y0, C_EDGE)

    def grid_at(self, x_step, y_step):
        x = math.ceil(self.xmin / x_step) * x_step
        while x <= self.xmax + 1e-9:
            px, _ = self.px(x, self.ymin)
            self.c.line(px, self.y0, px, self.y1, GRID)
            x += x_step
        y = math.ceil(self.ymin / y_step) * y_step
        while y <= self.ymax + 1e-9:
            _, py = self.px(self.xmin, y)
            self.c.line(self.x0, py, self.x1, py, GRID)
            y += y_step

    def plot(self, points, color, width=2, dashed=False):
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            self.c.line(*self.px(ax, ay), *self.px(bx, by), color, width,
                        dash=6 if dashed else 0)


def _nice_step(span: float, target: int) -> float:
    if span <= 0:
        return 1.0
    raw = span / max(1, target)
    magnitude = 10 ** math.floor(math.log10(raw))
    for factor in (1.0, 2.0, 2.5, 5.0, 10.0):
        if raw <= factor * magnitude:
            return factor * magnitude
    return 10.0 * magnitude


def _range(values, pad=0.08):
    lo, hi = min(values), max(values)
    if hi - lo < 1e-9:
        return lo - 1.0, hi + 1.0
    margin = (hi - lo) * pad
    return lo - margin, hi + margin


def _axis_labels(plot, x_step, y_step, suffix_x="", suffix_y="", digits=0,
                 digits_y=None):
    """x 轴和 y 轴的有效数字分开给：横轴是秒，纵轴常是几千牛，不能共用一个精度。"""
    if digits_y is None:
        digits_y = digits
    c = plot.c
    x = math.ceil(plot.xmin / x_step) * x_step
    while x <= plot.xmax + 1e-9:
        px, py = plot.px(x, plot.ymin)
        c.line(px, py, px, py + 3, MUTED)
        label = ("%." + str(digits) + "f") % (x + 0.0)
        c.text(px - c.text_width(label) / 2.0, py + 6, label, MUTED)
        x += x_step
    y = math.ceil(plot.ymin / y_step) * y_step
    while y <= plot.ymax + 1e-9:
        px, py = plot.px(plot.xmin, y)
        c.line(px - 3, py, px, py, MUTED)
        label = ("%." + str(digits_y) + "f") % (y + 0.0)
        c.text(px - c.text_width(label) - 5, py - 3, label, MUTED)
        y += y_step
    if suffix_x:
        c.text(plot.x1 - c.text_width(suffix_x), plot.y1 + 18, suffix_x, MUTED)
    if suffix_y:
        c.text(plot.x0 + 4, plot.y0 - 8, suffix_y, MUTED)


def _title(c, text, subtitle="", width=1180):
    c.text(30, 18, text, TEXT, 2)
    if subtitle:
        c.text(30, 40, subtitle, MUTED, 1)


def _legend(c, x, y, items):
    cursor = x
    for label, color in items:
        c.line(cursor, y + 4, cursor + 14, y + 4, color, 3)
        cursor = c.text(cursor + 18, y, label, MUTED) + 18


def _panel_title(c, x, y, text):
    c.text(x, y, text, TEXT)


def bump_figure(passive: SimResult, active: SimResult, width=1180, height=620):
    c = render.Canvas(width, height)
    c.fill_rect(0, 0, width, height, BG)
    _title(c, "SPEED BUMP  PASSIVE VS SEMI-ACTIVE",
           "20 KM/H, 50 MM BUMP, 0.4 M LONG")

    times = active.times()
    top_lo, top_hi = _range(passive.series("accel") + active.series("accel") + [0.0])
    top = Plot(c, 76, 84, width - 40, 330, 0, max(times), top_lo, top_hi)
    top.frame()
    x_step, y_step = _nice_step(max(times), 8), _nice_step(top_hi - top_lo, 5)
    top.grid_at(x_step, y_step)
    _axis_labels(top, x_step, y_step, "TIME / S", "BODY ACCEL / M/S2", digits=1)
    top.c.line(*top.px(0, 0), *top.px(max(times), 0), C_REF, 1, dash=8)
    top.plot(list(zip(times, passive.series("accel"))), C_PASSIVE, 2)
    top.plot(list(zip(times, active.series("accel"))), C_ACTIVE, 2)

    lo, hi = _range(passive.series("tire_front") + active.series("tire_front"))
    bottom = Plot(c, 76, 396, width - 40, 574, 0, max(times), lo, hi)
    bottom.frame()
    y2 = _nice_step(hi - lo, 5)
    bottom.grid_at(x_step, y2)
    _axis_labels(bottom, x_step, y2, "TIME / S", "FRONT AXLE LOAD / N", digits=1, digits_y=0)
    bottom.plot(list(zip(times, passive.series("tire_front"))), C_PASSIVE, 2)
    bottom.plot(list(zip(times, active.series("tire_front"))), C_ACTIVE, 2)

    _legend(c, width - 330, 16, [("PASSIVE", C_PASSIVE), ("SEMI-ACTIVE", C_ACTIVE)])
    c.text(width - 330, 46,
           "PEAK %.1f -> %.1f M/S2" % (passive.metrics["accel_peak"],
                                       active.metrics["accel_peak"]), MUTED)
    return c


def dive_figure(passive: SimResult, active: SimResult, width=1180, height=620):
    c = render.Canvas(width, height)
    c.fill_rect(0, 0, width, height, BG)
    _title(c, "BRAKE DIVE  PASSIVE VS SEMI-ACTIVE",
           "100 KM/H, 0.6 G DECELERATION")

    times = active.times()
    lo, hi = _range(passive.series("pitch") + active.series("pitch"))
    top = Plot(c, 76, 84, width - 40, 330, 0, max(times), lo, hi)
    top.frame()
    x_step, y_step = _nice_step(max(times), 8), _nice_step(hi - lo, 5)
    top.grid_at(x_step, y_step)
    _axis_labels(top, x_step, y_step, "TIME / S", "PITCH / DEG", digits=1)
    top.plot(list(zip(times, passive.series("pitch"))), C_PASSIVE, 2)
    top.plot(list(zip(times, active.series("pitch"))), C_ACTIVE, 2)

    front_p = [v * 1000.0 for v in passive.series("deflection_front")]
    front_a = [v * 1000.0 for v in active.series("deflection_front")]
    lo, hi = _range(front_p + front_a)
    bottom = Plot(c, 76, 396, width - 40, 574, 0, max(times), lo, hi)
    bottom.frame()
    y2 = _nice_step(hi - lo, 5)
    bottom.grid_at(x_step, y2)
    _axis_labels(bottom, x_step, y2, "TIME / S", "FRONT TRAVEL / MM", digits=1, digits_y=0)
    bottom.plot(list(zip(times, front_p)), C_PASSIVE, 2)
    bottom.plot(list(zip(times, front_a)), C_ACTIVE, 2)

    _legend(c, width - 330, 16, [("PASSIVE", C_PASSIVE), ("SEMI-ACTIVE", C_ACTIVE)])
    c.text(width - 330, 46,
           "PITCH %.2f -> %.2f DEG" % (passive.metrics["pitch_deg"],
                                       active.metrics["pitch_deg"]), MUTED)
    return c


def roll_figure(passive: SimResult, active: SimResult, width=1180, height=620):
    c = render.Canvas(width, height)
    c.fill_rect(0, 0, width, height, BG)
    _title(c, "CORNERING ROLL  PASSIVE VS SEMI-ACTIVE",
           "80 KM/H, 0.4 G LATERAL ACCELERATION")

    times = active.times()
    lo, hi = _range(passive.series("roll") + active.series("roll"))
    top = Plot(c, 76, 84, width - 40, 330, 0, max(times), lo, hi)
    top.frame()
    x_step, y_step = _nice_step(max(times), 8), _nice_step(hi - lo, 5)
    top.grid_at(x_step, y_step)
    _axis_labels(top, x_step, y_step, "TIME / S", "ROLL / DEG", digits=1)
    top.plot(list(zip(times, passive.series("roll"))), C_PASSIVE, 2)
    top.plot(list(zip(times, active.series("roll"))), C_ACTIVE, 2)

    # 下半部分看左右轮荷差：侧倾的物理后果全压在载荷转移上
    def transfer(result):
        return [f.tire_load[0] + f.tire_load[2]
                - f.tire_load[1] - f.tire_load[3] for f in result.frames]

    left_p, left_a = transfer(passive), transfer(active)
    lo, hi = _range(left_p + left_a)
    bottom = Plot(c, 76, 396, width - 40, 574, 0, max(times), lo, hi)
    bottom.frame()
    y2 = _nice_step(hi - lo, 5)
    bottom.grid_at(x_step, y2)
    _axis_labels(bottom, x_step, y2, "TIME / S", "LOAD TRANSFER / N", digits=1, digits_y=0)
    bottom.plot(list(zip(times, left_p)), C_PASSIVE, 2)
    bottom.plot(list(zip(times, left_a)), C_ACTIVE, 2)
    return c


def summary_figure(pairs, width=1180, height=520):
    """四个工况各自的关键指标，按被动 = 100% 归一。"""
    c = render.Canvas(width, height)
    c.fill_rect(0, 0, width, height, BG)
    _title(c, "KEY METRIC  PASSIVE = 100",
           "LOWER IS BETTER, SAME SCENARIO SIDE BY SIDE")

    rows = [
        ("BUMP PEAK ACCEL", pairs[0][0].metrics["accel_peak"],
         pairs[0][1].metrics["accel_peak"], "M/S2"),
        ("DIVE PITCH", pairs[1][0].metrics["pitch_deg"],
         pairs[1][1].metrics["pitch_deg"], "DEG"),
        ("ROLL ANGLE", pairs[2][0].metrics["roll_deg"],
         pairs[2][1].metrics["roll_deg"], "DEG"),
        ("ROUGH ROAD RMS", pairs[3][0].metrics["accel_rms"],
         pairs[3][1].metrics["accel_rms"], "M/S2"),
    ]
    top_y, row_h = 92, 104
    label_x, bar_x, bar_max = 60, 380, width - 220
    for index, (label, passive_value, active_value, unit) in enumerate(rows):
        y = top_y + index * row_h
        c.text(label_x, y + 8, label, TEXT)
        c.text(label_x, y + 26, "%s" % unit, MUTED)

        base = max(passive_value, 1e-6)
        for slot, (value, color, name) in enumerate(
                ((passive_value, C_PASSIVE, "PASSIVE"),
                 (active_value, C_ACTIVE, "SEMI"))):
            width_px = value / base * (bar_max - bar_x)
            y0 = y + slot * 26
            c.fill_rect(bar_x, y0, bar_x + max(2.0, width_px), y0 + 18, color)
            c.text(bar_x + width_px + 10, y0 + 4,
                   "%.2f  %s" % (value, name), MUTED)
        reduction = (1.0 - active_value / base) * 100.0
        c.text(bar_max + 120, y + 24, "-%.0f%%" % reduction, C_ACTIVE)

    c.text(60, height - 34,
           "SEMI-ACTIVE ONLY TRIMS THE TRANSIENT: STEADY ATTITUDE COMES FROM THE SPRINGS",
           MUTED)
    return c
