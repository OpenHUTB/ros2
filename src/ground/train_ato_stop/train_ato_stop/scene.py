"""图件绘制：速度-距离曲线、停车误差对比、运行过程动画。

沿用 render 里那套索引色画布与点阵字。点阵字只有大写字母和数字，
所以图上所有标注都用英文，中文留给文档正文。
"""
from __future__ import annotations

import math

from . import render
from .sim import SimResult

# 曲线配色
C_LIMIT = render.EDGE      # 限速台阶
C_ATP = render.STAND_ON    # ATP 允许速度
C_PLAN = render.C_A        # ATO 规划包络
C_RUN = render.C_B         # 实际速度
C_ZONE = render.SHADE      # 限速区底纹
C_ACCENT = render.C_C      # 对照色

TITLE_H = 42
FOOT_H = 22


class Plot:
    """把数据坐标映到像素坐标的极简画布区域。"""

    def __init__(self, canvas, x0, y0, x1, y1, xmin, xmax, ymin, ymax):
        self.c = canvas
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax

    def px(self, x: float, y: float) -> tuple[float, float]:
        fx = (x - self.xmin) / max(1e-9, self.xmax - self.xmin)
        fy = (y - self.ymin) / max(1e-9, self.ymax - self.ymin)
        return (self.x0 + fx * (self.x1 - self.x0),
                self.y1 - fy * (self.y1 - self.y0))

    def plot(self, points: list[tuple[float, float]], color: int, width: int = 1,
             dashed: bool = False) -> None:
        if len(points) < 2:
            return
        px = [self.px(x, y) for x, y in points]
        if not dashed:
            for (ax, ay), (bx, by) in zip(px, px[1:]):
                self.c.line(ax, ay, bx, by, color, width)
            return
        # 虚线：按固定像素节拍打断
        on, acc = True, 0.0
        for (ax, ay), (bx, by) in zip(px, px[1:]):
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-9:
                continue
            t = 0.0
            while t < seg:
                step = min(7.0 if on else 5.0, seg - t)
                if on:
                    self.c.line(ax + (bx - ax) * t / seg, ay + (by - ay) * t / seg,
                                ax + (bx - ax) * (t + step) / seg,
                                ay + (by - ay) * (t + step) / seg, color, width)
                t += step
                on = not on
                acc += step

    def frame(self, color: int = render.MUTED) -> None:
        self.c.fill_rect(self.x0, self.y0, self.x1, self.y0 + 1, color)
        self.c.fill_rect(self.x0, self.y1 - 1, self.x1, self.y1, color)
        self.c.fill_rect(self.x0, self.y0, self.x0 + 1, self.y1, color)
        self.c.fill_rect(self.x1 - 1, self.y0, self.x1, self.y1, color)

    def grid(self, nx: int, ny: int) -> None:
        for i in range(1, nx):
            x = self.xmin + (self.xmax - self.xmin) * i / nx
            px, _ = self.px(x, self.ymin)
            self.c.line(px, self.y0, px, self.y1, render.GRID)
        for j in range(1, ny):
            y = self.ymin + (self.ymax - self.ymin) * j / ny
            _, py = self.px(self.xmin, y)
            self.c.line(self.x0, py, self.x1, py, render.GRID)


def _nice_step(span: float, target: int = 6) -> float:
    if span <= 0:
        return 1.0
    raw = span / max(1, target)
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        if raw <= mag * mult:
            return mag * mult
    return mag * 10


def _axis_labels(plot: Plot, x_step: float, y_step: float, scale: int = 1,
                 suffix_x: str = '', suffix_y: str = '') -> None:
    """刻度值与轴标题。点阵字窄，标签一律横排。"""
    c = plot.c
    x = plot.xmin
    while x <= plot.xmax + 1e-6:
        px, py = plot.px(x, plot.ymin)
        c.line(px, py, px, py + 3, render.MUTED)
        label = f'{x:.0f}'
        w = c.text_width(label, scale)
        c.text(px - w / 2.0, py + 5, label, render.MUTED, scale)
        x += x_step
    y = plot.ymin
    while y <= plot.ymax + 1e-6:
        px, py = plot.px(plot.xmin, y)
        c.line(px - 3, py, px, py, render.MUTED)
        label = f'{y:.0f}'
        w = c.text_width(label, scale)
        c.text(px - w - 6, py - 3.5 * scale, label, render.MUTED, scale)
        y += y_step
    if suffix_x:
        c.text(plot.x1 - c.text_width(suffix_x, scale), plot.y1 + 16, suffix_x,
               render.MUTED, scale)
    if suffix_y:
        c.text(4, plot.y0 - 12, suffix_y, render.MUTED, scale)


def _grade_ribbon(c: render.Canvas, result: SimResult, plot: Plot) -> None:
    """坐标轴下方的坡度色带。没有它，坡道图和平直道图看起来几乎一样。"""
    y0, y1 = plot.y1 + 30, plot.y1 + 42
    for seg in result.segments:
        a, _ = plot.px(seg.start, 0.0)
        b, _ = plot.px(min(seg.end, result.stop_x), 0.0)
        if seg.grade > 0.01:
            color = render.GIVE_WAY
        elif seg.grade < -0.01:
            color = render.STAND_ON
        else:
            color = render.SHADE
        c.fill_rect(a, y0, b, y1, color)
        if abs(seg.grade) <= 0.01:
            continue
        for label in (f'{seg.grade:+.0f} PERMILLE', f'{seg.grade:+.0f}'):
            if c.text_width(label, 1) < (b - a) - 8:
                c.text(a + 4, y0 + 4, label, render.WHITE, 1)
                break


def speed_profile_chart(result: SimResult, title: str, subtitle: str,
                        width: int = 900, height: int = 560,
                        show_limit_bands: bool = True) -> render.Canvas:
    """速度-距离曲线：限速台阶、ATP 允许速度、ATO 规划包络、实际速度。"""
    c = render.Canvas(width, height)
    c.text(14, 10, title, render.TEXT, 2)
    c.text(14, 30, subtitle, render.MUTED, 1)

    xmax = result.stop_x * 1.02
    vmax_kph = max(max(result.profile.v_limit), 40.0 / 3.6) * 3.6 * 1.12
    box_top, box_bottom = TITLE_H + 16, height - FOOT_H - 60
    plot = Plot(c, 78, box_top, width - 26, box_bottom, 0.0, xmax, 0.0, vmax_kph)

    plot.grid(8, 5)

    # 限速区底纹：比限速低的段整片铺色，一眼看出哪段要减速
    if show_limit_bands:
        prev_x, prev_v = 0.0, result.profile.limit_at(0.0) * 3.6
        for i, x in enumerate(result.profile.x):
            v = result.profile.v_limit[i] * 3.6
            if abs(v - prev_v) > 1e-6:
                if prev_v < max(result.profile.v_limit) * 3.6 - 1e-6:
                    a = plot.px(prev_x, 0.0)
                    b = plot.px(x, prev_v)
                    c.fill_rect(a[0], b[1], b[0], a[1], C_ZONE)
                prev_x, prev_v = x, v

    # 限速台阶
    pts = [(x, v * 3.6) for x, v in zip(result.profile.x, result.profile.v_limit)]
    plot.plot(pts, C_LIMIT, 2)
    # ATP 允许速度
    plot.plot([(x, v * 3.6) for x, v in zip(result.atp_profile.x, result.atp_profile.v_envelope)],
              C_ATP, 1, dashed=True)
    # ATO 规划包络：画粗一点，否则会被贴上去的实际速度完全盖住
    plot.plot([(x, v * 3.6) for x, v in zip(result.profile.x, result.profile.v_envelope)],
              C_PLAN, 3, dashed=True)
    # 实际速度
    plot.plot([(f.x, f.v * 3.6) for f in result.frames], C_RUN, 2)

    plot.frame()

    x_step = _nice_step(xmax, 8)
    y_step = _nice_step(vmax_kph, 5)
    _axis_labels(plot, x_step, y_step, 1, 'DISTANCE (M)', 'SPEED (KM/H)')
    if result.segments:
        _grade_ribbon(c, result, plot)

    # 停车点标记
    sx, _ = plot.px(result.stop_x, 0.0)
    c.line(sx, plot.y0, sx, plot.y1, render.WARN, 1)

    # 图例
    lx, ly = width - 236, box_top + 10
    c.fill_rect(lx - 8, ly - 6, width - 18, ly + 56, render.LEGEND_BG)
    entries = [('LIMIT', C_LIMIT), ('ATP PERMIT', C_ATP),
               ('ATO PLAN', C_PLAN), ('ACTUAL', C_RUN)]
    for i, (label, color) in enumerate(entries):
        yy = ly + i * 13
        c.fill_rect(lx, yy, lx + 16, yy + 3, color)
        c.text(lx + 22, yy - 2, label, render.TEXT, 1)

    err = result.metrics['stop_error']
    verdict = f"STOP ERROR {err:+.2f} M"
    c.text(14, height - FOOT_H + 4, verdict,
           render.C_C if abs(err) <= 0.3 else render.WARN, 1)
    t = result.metrics['run_time']
    c.text(width - 200, height - FOOT_H + 4,
           f"T={t:.0f} S  E={result.metrics['energy_kwh']:.0f} KWH", render.MUTED, 1)
    return c


def _error_panel(c: render.Canvas, x0: int, x1: int, y_top: int, row_h: int,
                 vmin: float, vmax: float, rows: list, tol: float | None) -> None:
    """一个量程下的一组误差横条。"""
    span = vmax - vmin

    def px(value: float) -> float:
        return x0 + (value - vmin) / span * (x1 - x0)

    zero = px(0.0)
    bottom = y_top + row_h * len(rows)

    if tol:
        for edge in (-tol, tol):
            c.line(px(edge), y_top + 2, px(edge), bottom, render.C_C)

    for i, (label, value, color) in enumerate(rows):
        yc = y_top + (i + 0.5) * row_h
        half = row_h * 0.28
        xv = px(max(vmin, min(vmax, value)))
        lo, hi = sorted((zero, xv))
        c.fill_rect(lo, yc - half, hi, yc + half, color)
        c.text(x0 - 142, yc - 3.5, label, render.TEXT, 1)
        txt = f'{value:+.2f} M'
        tx = hi + 6 if xv >= zero else lo - c.text_width(txt, 1) - 6
        c.text(tx, yc - 3.5, txt, color, 1)

    c.line(x0, bottom, x1, bottom, render.MUTED)
    c.line(zero, y_top + 2, zero, bottom, render.TEXT)


def stop_error_chart(results: list[SimResult], width: int = 900,
                     height: int = 320) -> render.Canvas:
    """停车误差对比。

    量程差得太多（0.02 m 对 76 m），放在一根轴上小误差的柱子宽度是 0，
    什么也看不出来。所以拆成上下两块，各用自己的量程，并在标题里写清楚。
    """
    c = render.Canvas(width, height)
    c.text(14, 10, 'STOP ACCURACY', render.TEXT, 2)
    c.text(14, 30, 'BAR = FINAL POSITION ERROR, POSITIVE MEANS OVERSHOOT', render.MUTED, 1)

    tol = 0.3
    ato = [r for r in results if r.name != 'late_brake']
    bad = [r for r in results if r.name == 'late_brake']

    c.text(14, 56, 'ATO RUNS  (SCALE -0.6 TO +0.6 M, GREEN LINES = 0.3 M TOLERANCE)',
           render.MUTED, 1)
    rows = [(r.title_en, r.metrics['stop_error'],
             render.C_C if abs(r.metrics['stop_error']) <= tol else render.WARN)
            for r in ato]
    _error_panel(c, 170, width - 60, 68, 34, -0.6, 0.6, rows, tol)

    if bad:
        c.text(14, 196, 'REFERENCE RUN  (SCALE 0 TO 90 M, SAME UNIT)', render.MUTED, 1)
        rows_bad = [(r.title_en, r.metrics['stop_error'], render.WARN) for r in bad]
        peak = max(90.0, max(abs(v) for _, v, _ in rows_bad) * 1.15)
        _error_panel(c, 170, width - 60, 208, 40, 0.0, peak, rows_bad, None)
        for step in range(0, int(peak) + 1, 20):
            x = 170 + step / peak * (width - 60 - 170)
            c.line(x, 248, x, 252, render.MUTED)
            c.text(x - c.text_width(str(step), 1) / 2.0, 256, str(step), render.MUTED, 1)
        label = 'OVERSHOOT (M)'
        c.text(width - 60 - c.text_width(label, 1), 272, label, render.MUTED, 1)

    c.text(14, height - 18,
           'THE THREE ATO RUNS LAND INSIDE THE PLATFORM MARK; THE REFERENCE RUN STOPS '
           'BEYOND THE PLATFORM END', render.MUTED, 1)
    return c


def _route_strip(c: render.Canvas, result: SimResult, x: float, v: float,
                 box: tuple[int, int, int, int]) -> None:
    """线路示意：限速区色带 + 车站 + 列车方块。"""
    x0, y0, x1, y1 = box
    c.fill_rect(x0, y0, x1, y1, render.SHADE)
    span = result.stop_x
    prev_x, prev_v = 0.0, result.profile.limit_at(0.0)
    vmax = max(result.profile.v_limit)

    def seg_color(limit: float) -> int:
        return render.EDGE if limit >= vmax - 1e-6 else render.C_ATP

    for i, xx in enumerate(result.profile.x):
        lim = result.profile.v_limit[i]
        if abs(lim - prev_v) > 1e-6:
            a = x0 + (prev_x / span) * (x1 - x0)
            b = x0 + (xx / span) * (x1 - x0)
            c.fill_rect(a, y0 + 8, b, y1 - 8, seg_color(prev_v))
            prev_x, prev_v = xx, lim
    a = x0 + (prev_x / span) * (x1 - x0)
    c.fill_rect(a, y0 + 8, x1, y1 - 8, seg_color(prev_v))

    # 车站
    sx = x0 + (result.stop_x / span) * (x1 - x0)
    c.fill_rect(sx - 2, y0 - 4, sx + 2, y1 + 4, render.WARN)
    label = 'STATION B'
    c.text(min(sx - 30, c.w - c.text_width(label, 1) - 4), y1 + 6, label, render.WARN, 1)

    # 列车
    tx = x0 + (x / span) * (x1 - x0)
    c.fill_rect(tx - 12, y0 + 2, tx + 2, y1 - 2, render.HULL)
    c.fill_rect(tx - 10, y0 + 5, tx, y0 + 9, render.WHITE)


def render_animation(result: SimResult, frames: int = 60,
                     width: int = 760, height: int = 444) -> list[render.Canvas]:
    """运行过程动画：上半是速度-距离曲线带游标，下半是线路与列车位置。"""
    total = len(result.frames)
    out = []
    vmax_kph = max(result.profile.v_limit) * 3.6 * 1.15
    xmax = result.stop_x * 1.02

    for k in range(frames):
        idx = int(round(k * (total - 1) / max(1, frames - 1)))
        frame = result.frames[idx]
        c = render.Canvas(width, height)

        c.text(12, 8, result.title_en, render.TEXT, 2)
        c.text(12, 26, f'T = {frame.t:5.1f} S', render.MUTED, 1)
        spd = f'{frame.v * 3.6:5.1f} KM/H'
        c.text(width - c.text_width(spd, 2) - 12, 8, spd, render.TEXT, 2)
        dist = f'{max(0.0, result.stop_x - frame.x):5.0f} M TO STOP'
        c.text(width - c.text_width(dist, 1) - 12, 30, dist, render.MUTED, 1)

        top, bottom = 50, 330
        plot = Plot(c, 62, top, width - 18, bottom, 0.0, xmax, 0.0, vmax_kph)
        plot.grid(6, 4)
        plot.plot([(x, v * 3.6) for x, v in zip(result.profile.x, result.profile.v_limit)],
                  C_LIMIT, 1)
        plot.plot([(x, v * 3.6) for x, v in zip(result.profile.x, result.profile.v_envelope)],
                  C_PLAN, 1, dashed=True)
        # 已经走过的部分画实线，没走的画淡色
        passed = [(f.x, f.v * 3.6) for f in result.frames[:idx + 1]]
        plot.plot(passed, C_RUN, 2)
        ahead = [(f.x, f.v * 3.6) for f in result.frames[idx:]]
        if len(ahead) > 1:
            plot.plot(ahead, render.EDGE, 1)
        plot.frame()
        _axis_labels(plot, _nice_step(xmax, 6), _nice_step(vmax_kph, 4), 1,
                     '', 'SPEED (KM/H)')

        # 游标
        cx, cy = plot.px(frame.x, frame.v * 3.6)
        c.disc(cx, cy, 4, render.WARN)
        c.line(cx, plot.y0, cx, plot.y1, render.SHADE)

        _route_strip(c, result, frame.x, frame.v, (62, 372, width - 18, 404))

        note = frame.note
        if frame.atp == 'intervention':
            note = 'ATP EMERGENCY BRAKE'
        c.text(62, 418, note, render.WARN if frame.atp != 'normal' else render.MUTED, 1)
        c.text(width - 210, 418, f"CMD {frame.cmd:+.2f} M/S2", render.MUTED, 1)
        out.append(c)
    return out
