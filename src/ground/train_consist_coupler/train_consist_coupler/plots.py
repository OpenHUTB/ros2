"""把仿真结果画成 PNG（坐标轴、刻度、图例都是手写的，不依赖 matplotlib）。"""

import math

from .render import (AXIS, BLUE, Canvas, GRID, GREEN, INK, MUTED, ORANGE,
                     PURPLE, RED, TEAL)

W, H = 960, 600
LEFT, RIGHT, TOP, BOTTOM = 96, 48, 84, 82


class Axes:
    def __init__(self, canvas, box, xlim, ylim, title="", xlabel="", unit="",
                 subtitle=""):
        self.cv = canvas
        self.x0, self.y0, self.x1, self.y1 = box
        self.xlim = xlim
        self.ylim = ylim
        self.title = title
        self.xlabel = xlabel
        self.unit = unit
        self.subtitle = subtitle

    # ---------- 坐标变换 ----------
    def X(self, value):
        span = self.xlim[1] - self.xlim[0]
        return self.x0 + (value - self.xlim[0]) / span * (self.x1 - self.x0)

    def Y(self, value):
        span = self.ylim[1] - self.ylim[0]
        return self.y1 - (value - self.ylim[0]) / span * (self.y1 - self.y0)

    # ---------- 装饰 ----------
    def draw_texts(self):
        if self.title:
            self.cv.text_center(self.title, (self.x0 + self.x1) // 2, 14, INK, 3)
        if self.subtitle:
            self.cv.text_center(self.subtitle, (self.x0 + self.x1) // 2, 42, MUTED, 2)
        if self.xlabel:
            self.cv.text_center(self.xlabel, (self.x0 + self.x1) // 2,
                                self.y1 + 46, MUTED, 2)
        if self.unit:
            self.cv.text_right(self.unit, self.x0 - 10, self.y0 - 22, MUTED, 2)

    def frame(self):
        self.cv.hline(self.x0, self.x1, self.y1, AXIS)
        self.cv.vline(self.x0, self.y0, self.y1, AXIS)

    def grid(self, nx=6, ny=6):
        for i in range(nx + 1):
            if nx:
                x = self.x0 + (self.x1 - self.x0) * i / nx
                self.cv.vline(x, self.y0, self.y1, GRID)
        for j in range(ny + 1):
            if ny:
                y = self.y0 + (self.y1 - self.y0) * j / ny
                self.cv.hline(self.x0, self.x1, y, GRID)

    def _visible(self, value, bounds):
        low, high = bounds
        span = high - low
        return low - span * 1e-6 <= value <= high + span * 1e-6

    def xticks(self, values, digits=0):
        for value in values:
            if not self._visible(value, self.xlim):
                continue
            x = self.X(value)
            self.cv.vline(x, self.y1, self.y1 + 6, AXIS)
            self.cv.text_center(("%." + str(digits) + "f") % value, x,
                                self.y1 + 12, MUTED, 2)

    def yticks(self, values, digits=0):
        for value in values:
            if not self._visible(value, self.ylim):
                continue
            y = self.Y(value)
            self.cv.hline(self.x0 - 6, self.x0, y, AXIS)
            self.cv.text_right(("%." + str(digits) + "f") % value, self.x0 - 10,
                               y - 7, MUTED, 2)

    # ---------- 数据 ----------
    def plot(self, xs, ys, color, width=2, dash=0):
        xs = list(xs)
        ys = list(ys)
        if len(xs) > 1600:
            stride = max(1, len(xs) // 1600)
            xs, ys = xs[::stride], ys[::stride]
        period = dash * 2
        px = py = None
        travelled = 0.0
        for x, y in zip(xs, ys):
            cx, cy = self.X(x), self.Y(y)
            if px is None:
                px, py = cx, cy
                continue
            segment = math.hypot(cx - px, cy - py)
            if segment <= 0.01:
                px, py = cx, cy
                continue
            if dash <= 0:
                self.cv.line(px, py, cx, cy, color, width)
            else:
                step = 0.0
                while step < segment:
                    on = (travelled + step) % period < dash
                    length = min(segment - step, 1.5)
                    if on:
                        t0 = step / segment
                        t1 = (step + length) / segment
                        self.cv.line(px + (cx - px) * t0, py + (cy - py) * t0,
                                     px + (cx - px) * t1, py + (cy - py) * t1,
                                     color, width)
                    step += length
            travelled += segment
            px, py = cx, cy

    def hline_value(self, value, color, dash=6, width=2):
        if not self._visible(value, self.ylim):
            return
        y = self.Y(value)
        if dash:
            self.cv.dashed(self.x0, y, self.x1, y, color, dash, 5, width)
        else:
            self.cv.hline(self.x0, self.x1, y, color)

    def vline_value(self, value, color, dash=6, width=2):
        if not self._visible(value, self.xlim):
            return
        x = self.X(value)
        if dash:
            self.cv.dashed(x, self.y0, x, self.y1, color, dash, 5, width)
        else:
            self.cv.vline(x, self.y0, self.y1, color)

    def bar(self, center, value, width_px, color):
        cx = int(self.X(center))
        y_zero = self.Y(0.0)
        y_value = self.Y(max(min(value, self.ylim[1]), self.ylim[0]))
        top = min(y_zero, y_value)
        height = max(1, abs(y_value - y_zero))
        self.cv.fill(cx - width_px // 2, top, width_px, height, color)

    # ---------- 图例 ----------
    def legend(self, entries, x_anchor=None, y_anchor=None, scale=2):
        row = 24
        width = max(self.cv.text_width(label, scale) + 42 for label, _c, _s in entries)
        x = (self.x1 - width - 12) if x_anchor is None else x_anchor
        y = (self.y0 + 12) if y_anchor is None else y_anchor
        self.cv.fill(x - 8, y - 6, width + 14, row * len(entries) + 8, (255, 255, 255))
        self.cv.rect(x - 8, y - 6, width + 14, row * len(entries) + 8, GRID)
        for i, (label, color, style) in enumerate(entries):
            cy = y + i * row + 8
            if style == "dash":
                self.cv.dashed(x, cy, x + 26, cy, color, 5, 4, 2)
            elif style == "bar":
                self.cv.fill(x + 4, cy - 5, 18, 10, color)
            else:
                self.cv.line(x, cy, x + 26, cy, color, 3)
            self.cv.text(label, x + 34, cy - 7, INK, scale)


def new_canvas():
    return Canvas(W, H, (255, 255, 255))


def nice_ticks(low, high, count=5):
    if high <= low:
        high = low + 1.0
    raw = (high - low) / count
    magnitude = 10 ** math.floor(math.log10(raw))
    step = magnitude * 10
    for multiple in (1, 2, 2.5, 5, 10):
        if raw <= magnitude * multiple:
            step = magnitude * multiple
            break
    start = math.floor(low / step) * step
    ticks = []
    value = start
    while value <= high + step * 0.5:
        ticks.append(round(value, 6))
        value += step
    return ticks


# ---------------------------------------------------------------- 图 1
def figure_force_route(results, path, scenario="consist_head_first"):
    """车钩力沿线路的分布：起步、制动起点、停车末段各出现一次。"""
    cv = new_canvas()
    hist = results[scenario]["history"]
    forces = list(zip(*hist["couplers"]))
    flat = [value / 1e3 for row in forces for value in row]
    low, high = min(flat), max(flat)
    margin = max(6.0, (high - low) * 0.12)
    ticks = nice_ticks(low - margin, high + margin, 5)

    ax = Axes(cv, (LEFT, TOP, W - RIGHT, H - BOTTOM), (0, 2400),
              (min(ticks), max(ticks)),
              title="COUPLER FORCE (KN) ALONG THE ROUTE",
              xlabel="HEAD POSITION (M)", unit="",
              subtitle="4 CARS, HEAD-TO-TAIL 60 MS; DASHED: BRAKE ONSET")

    ax.grid(6, 5)
    ax.xticks([0, 480, 960, 1440, 1920, 2400], 0)
    ax.yticks(ticks, 0)
    ax.hline_value(0.0, AXIS, dash=0, width=1)
    ax.vline_value(results[scenario]["brake_start"], MUTED, dash=5)

    colors = (BLUE, RED, GREEN, ORANGE, PURPLE, TEAL)
    for i, row in enumerate(forces):
        ax.plot(hist["head"], [value / 1e3 for value in row],
                colors[i % len(colors)], 2)

    ax.frame()
    ax.legend([("COUPLER %d" % (i + 1), colors[i % len(colors)], "solid")
               for i in range(len(forces))])
    ax.draw_texts()
    cv.save_png(path)


# ---------------------------------------------------------------- 图 2
def figure_coupler_history(results, path, scenario="consist_head_first"):
    cv = new_canvas()
    hist = results[scenario]["history"]
    forces = list(zip(*hist["couplers"]))
    brake_time = results[scenario]["brake_time"]
    start = max(0.0, brake_time - 6.0)
    end = hist["t"][-1]

    indices = [i for i, t in enumerate(hist["t"]) if t >= start]
    flat = [row[i] / 1e3 for row in forces for i in indices]
    low, high = min(flat), max(flat)
    margin = max(5.0, (high - low) * 0.15)
    ticks = nice_ticks(low - margin, high + margin, 5)
    ylim = (min(ticks), max(ticks))

    ax = Axes(cv, (LEFT, TOP, W - RIGHT, H - BOTTOM), (start, end), ylim,
              title="COUPLER FORCE (KN): FINAL BRAKING AND STOP",
              xlabel="TIME (S)", unit="",
              subtitle="4 CARS, HEAD-TO-TAIL 60 MS; POSITIVE = BUFF")

    ax.grid(6, 5)
    ax.xticks(nice_ticks(start, end, 6), 0)
    ax.yticks(ticks, 0)
    ax.hline_value(0.0, AXIS, dash=0, width=1)
    ax.vline_value(brake_time, MUTED, dash=5)

    colors = (BLUE, RED, GREEN, ORANGE, PURPLE, TEAL)
    for i, row in enumerate(forces):
        xs = [t for t in hist["t"] if t >= start]
        ys = [value / 1e3 for t, value in zip(hist["t"], row) if t >= start]
        ax.plot(xs, ys, colors[i % len(colors)], 2)

    ax.frame()
    ax.legend([("COUPLER %d" % (i + 1), colors[i % len(colors)], "solid")
               for i in range(len(forces))], x_anchor=ax.x0 + 14, y_anchor=ax.y0 + 12)
    ax.draw_texts()
    cv.save_png(path)


# ---------------------------------------------------------------- 图 3
def figure_peak_bars(results, order, labels, path):
    cv = new_canvas()
    buffs = [results[name]["peak_buff"] / 1e3 for name in order]
    drafts = [results[name]["peak_draft"] / 1e3 for name in order]
    top = max(max(buffs), -min(drafts)) * 1.15
    ax = Axes(cv, (LEFT, TOP, W - RIGHT, H - BOTTOM), (0, len(order)),
              (-top, top), title="PEAK COUPLER FORCE (KN) BY SCENARIO",
              xlabel="", unit="",
              subtitle="BUFF ABOVE ZERO, DRAFT BELOW ZERO")

    ticks = nice_ticks(-top, top, 5)
    ax.grid(0, 5)
    ax.yticks(ticks, 0)
    ax.hline_value(0.0, AXIS, dash=0, width=2)
    for i, name in enumerate(order):
        ax.bar(i + 0.36, buffs[i], 28, RED)
        ax.bar(i + 0.64, drafts[i], 28, BLUE)
        ax.cv.text_center(labels[name], ax.X(i + 0.5), ax.y1 + 12, MUTED, 2)
        if buffs[i] > 1.0:
            ax.cv.text_center("%.0f" % buffs[i], ax.X(i + 0.36),
                              ax.Y(buffs[i]) - 20, RED, 2)
        if drafts[i] < -1.0:
            ax.cv.text_center("%.0f" % drafts[i], ax.X(i + 0.64),
                              ax.Y(drafts[i]) + 6, BLUE, 2)

    ax.frame()
    ax.legend([("BUFF", RED, "bar"), ("DRAFT", BLUE, "bar")],
              x_anchor=ax.x0 + 14, y_anchor=ax.y0 + 12)
    ax.draw_texts()
    cv.save_png(path)


# ---------------------------------------------------------------- 图 4
def figure_stop_error(results, order, labels, path):
    cv = new_canvas()
    values = [results[name]["stop_error"] for name in order]
    span = max(0.35, max(abs(v) for v in values) * 1.35)
    ax = Axes(cv, (LEFT, TOP, W - RIGHT, H - BOTTOM), (0, len(order)),
              (-span, span), title="STOP ERROR (M) AT THE 2400 M MARKER",
              xlabel="", unit="",
              subtitle="DASHED LINES: 0.3 M STOPPING WINDOW")

    ticks = nice_ticks(-span, span, 4)
    ax.grid(0, 4)
    ax.yticks(ticks, 2)
    ax.hline_value(0.3, ORANGE, dash=6)
    ax.hline_value(-0.3, ORANGE, dash=6)
    ax.hline_value(0.0, AXIS, dash=0, width=2)
    for i, value in enumerate(values):
        ax.bar(i + 0.5, value, 36, GREEN if abs(value) <= 0.3 else RED)
        above = value >= 0
        ax.cv.text_center("%+0.2f" % value, ax.X(i + 0.5),
                          ax.Y(value) - 20 if above else ax.Y(value) + 6, INK, 2)
        ax.cv.text_center(labels[order[i]], ax.X(i + 0.5), ax.y1 + 12, MUTED, 2)

    ax.frame()
    ax.legend([("WITHIN 0.3 M", GREEN, "bar"), ("OUT OF WINDOW", RED, "bar")],
              x_anchor=ax.x0 + 14, y_anchor=ax.y0 + 12)
    ax.draw_texts()
    cv.save_png(path)


# ---------------------------------------------------------------- 图 5
def figure_sweep(sweep, cars, delays, path):
    cv = new_canvas()
    cv.text_center("PEAK COUPLER FORCE VS COMMAND DELAY", W // 2, 14, INK, 3)
    cv.text_center("SAME ROUTE AND SPEED PROFILE; DELAY IS PER CAR", W // 2, 42,
                   MUTED, 2)

    box_left = (88, 108, 468, 452)
    box_right = (560, 108, 928, 452)
    colors = {4: BLUE, 6: ORANGE, 8: RED}
    all_values = [abs(row[key]) / 1e3 for row in sweep.values()
                  for key in ("buff", "draft")]
    top = max(all_values) * 1.15
    ticks = nice_ticks(0, top, 5)
    top = max(ticks)
    panels = ((box_left, "BUFF, KN (TAIL FIRST)", "buff"),
              (box_right, "DRAFT, KN (HEAD FIRST)", "draft"))

    for box, subtitle, key in panels:
        ax = Axes(cv, box, (0, max(delays) * 1e3 + 14), (0, top))
        ax.grid(5, 5)
        ax.xticks(nice_ticks(0, max(delays) * 1e3, 5), 0)
        ax.yticks(ticks, 0)
        for n in cars:
            xs = [d * 1e3 for d in delays]
            ys = [abs(sweep[(n, d)][key]) / 1e3 for d in delays]
            ax.plot(xs, ys, colors[n], 3 if n == 8 else 2)
        ax.frame()
        ax.cv.text_center(subtitle, (box[0] + box[2]) // 2, 76, MUTED, 2)
        ax.legend([("%d CARS" % n, colors[n], "solid") for n in cars],
                  x_anchor=box[0] + 10, y_anchor=box[1] + 12)

    cv.text_center("DELAY PER CAR (MS)", W // 2, 486, MUTED, 2)
    cv.save_png(path)
