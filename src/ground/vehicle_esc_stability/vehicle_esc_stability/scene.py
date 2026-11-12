"""图件绘制：轨迹对比、状态曲线、制动指令、指标对比与俯视动画。

沿用 render 里的索引色画布与点阵字。点阵字只有大写字母和数字，
所以图上标注一律用英文，中文说明留给文档正文。
"""
from __future__ import annotations

import math

from . import render
from .driver import ReferencePath
from .sim import SimResult

C_ON = render.C_A       # ESC 开
C_OFF = render.C_B      # ESC 关
C_REF = render.C_C      # 参考
C_WARN = render.WARN
C_MUTED = render.MUTED
C_TEXT = render.TEXT

WHEEL_COLORS = (render.C_A, render.C_B, render.C_C, render.STAND_ON)

TITLE_H = 40
FOOT_H = 20


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
        on = True
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

    def grid_at(self, x_step: float, y_step: float) -> None:
        """按刻度的实际位置画网格线，和坐标标注对齐。"""
        x = math.ceil(self.xmin / x_step - 1e-9) * x_step
        while x < self.xmax - 1e-9:
            px, _ = self.px(x, self.ymin)
            if self.x0 + 1 < px < self.x1 - 1:
                self.c.line(px, self.y0, px, self.y1, render.GRID)
            x += x_step
        y = math.ceil(self.ymin / y_step - 1e-9) * y_step
        while y < self.ymax - 1e-9:
            _, py = self.px(self.xmin, y)
            if self.y0 + 1 < py < self.y1 - 1:
                self.c.line(self.x0, py, self.x1, py, render.GRID)
            y += y_step


def _nice_step(span: float, target: int = 6) -> float:
    if span <= 0:
        return 1.0
    raw = span / max(1, target)
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        if raw <= mag * mult:
            return mag * mult
    return mag * 10


def _range(values: list[float], pad: float = 0.12) -> tuple[float, float]:
    """按数据范围给出带留白的坐标轴上下限。"""
    if not values:
        return -1.0, 1.0
    lo, hi = min(values), max(values)
    span = max(hi - lo, 1e-6)
    lo -= span * pad
    hi += span * pad
    step = _nice_step(hi - lo, 5)
    return math.floor(lo / step) * step, math.ceil(hi / step) * step


def _axis_labels(plot: Plot, x_step: float, y_step: float, scale: int = 1,
                 suffix_x: str = '', suffix_y: str = '', digits: int = 0) -> None:
    c = plot.c
    x = math.ceil(plot.xmin / x_step - 1e-9) * x_step
    while x <= plot.xmax + 1e-6:
        px, py = plot.px(x, plot.ymin)
        c.line(px, py, px, py + 3, render.MUTED)
        label = f'{x + 0.0:g}'
        c.text(px - c.text_width(label, scale) / 2.0, py + 5, label, render.MUTED, scale)
        x += x_step
    y = math.ceil(plot.ymin / y_step - 1e-9) * y_step
    while y <= plot.ymax + 1e-6:
        px, py = plot.px(plot.xmin, y)
        c.line(px - 3, py, px, py, render.MUTED)
        label = f'{y:.{digits}f}'
        c.text(px - c.text_width(label, scale) - 6, py - 3.5 * scale, label,
               render.MUTED, scale)
        y += y_step
    if suffix_x:
        c.text(plot.x1 - c.text_width(suffix_x, scale), plot.y1 + 14, suffix_x,
               render.MUTED, scale)
    if suffix_y:
        c.text(plot.x0 + 4, plot.y0 - 13, suffix_y, render.MUTED, scale)


def _legend(c: render.Canvas, x: float, y: float, items: list[tuple[str, int]]) -> None:
    cursor = x
    for label, color in items:
        c.line(cursor, y + 4, cursor + 16, y + 4, color, 3)
        cursor = c.text(cursor + 20, y, label, render.MUTED) + 14


def _title(c: render.Canvas, text: str, sub: str = '') -> None:
    c.text(20, 14, text, C_TEXT, 2)
    if sub:
        c.text(20, 30, sub, C_MUTED, 1)


def draw_car(c: render.Canvas, cx: float, cy: float, yaw: float, color: int,
             length_px: float = 26.0, width_px: float = 12.0) -> None:
    """在像素坐标上画一个带朝向的车身（屏幕 y 向下，所以绕 -yaw 旋转）。"""
    half_l, half_w = length_px / 2.0, width_px / 2.0
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    corners = []
    for dx, dy in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)):
        corners.append((cx + dx * cos_y + dy * sin_y, cy - dx * sin_y + dy * cos_y))
    c.poly(corners, color)
    nose = []
    for dx, dy in ((half_l + 7.0, 0.0), (half_l - 1.0, 4.0), (half_l - 1.0, -4.0)):
        nose.append((cx + dx * cos_y + dy * sin_y, cy - dx * sin_y + dy * cos_y))
    c.poly(nose, color)


# ---------------------------------------------------------------- 图 1 轨迹
def trajectory_figure(off: SimResult, on: SimResult, width: int = 1180,
                      height: int = 660) -> render.Canvas:
    """双移线轨迹对比，按 0.8 s 间隔画车身姿态。"""
    path: ReferencePath = on.scenario.path
    c = render.Canvas(width, height)
    _title(c, f'{on.scenario.title_for_render}  ESC OFF VS ON',
           f'{on.scenario.initial_speed * 3.6:.0f} KM/H, MU {on.scenario.road.mean:.1f}, '
           f'LANE OFFSET {path.lane_width:.1f} M')

    xs = [p[0] for p in path.points] + [f.state.x for f in on.frames]
    ys = [p[1] for p in path.points] + [f.state.y for f in off.frames] + \
         [f.state.y for f in on.frames]
    xmax = max(xs) * 1.02
    ymin, ymax = min(ys) - 5.0, max(ys) + 5.0
    plot = Plot(c, 74, 66, width - 30, height - 78, 0.0, xmax, ymin, ymax)
    x_step, y_step = _nice_step(xmax, 8), _nice_step(ymax - ymin, 6)
    plot.grid_at(x_step, y_step)
    _axis_labels(plot, x_step, y_step, 1, 'X / M', 'Y / M')

    for lane_y in (-3.5, 0.0, 3.5):
        plot.plot([(0.0, lane_y), (xmax, lane_y)], render.C_LANE, 1, dashed=True)
    plot.plot(path.points, C_REF, 2)

    def draw_run(result: SimResult, color: int, every: float = 0.8) -> None:
        plot.plot([(f.state.x, f.state.y) for f in result.frames], color, 3)
        next_t = 0.0
        for frame in result.frames:
            if frame.t + 1e-9 >= next_t:
                px, py = plot.px(frame.state.x, frame.state.y)
                draw_car(c, px, py, frame.state.yaw, color)
                next_t += every

    draw_run(off, C_OFF)
    draw_run(on, C_ON)

    _legend(c, 560, 14, [('ESC OFF', C_OFF), ('ESC ON', C_ON), ('REFERENCE', C_REF)])
    c.text(64, height - 46,
           f'ESC OFF: MAX BETA {off.metrics["max_beta_deg"]:.1f} DEG, MAX DEVIATION '
           f'{off.metrics["max_cross_track_m"]:.1f} m', C_OFF)
    c.text(64, height - 34,
           f'ESC ON : MAX BETA {on.metrics["max_beta_deg"]:.1f} DEG, MAX DEVIATION '
           f'{on.metrics["max_cross_track_m"]:.2f} m', C_ON)
    return c


# ---------------------------------------------------------------- 图 2 状态
def state_figure(off: SimResult, on: SimResult, width: int = 1180,
                 height: int = 720) -> render.Canvas:
    """横摆角速度跟踪与侧偏角。"""
    c = render.Canvas(width, height)
    _title(c, f'{off.scenario.title_for_render}  YAW RATE AND SIDESLIP',
           'GREY DASHED: REFERENCE YAW RATE')

    yaw_lo, yaw_hi = _range(off.series('yaw_rate') + on.series('yaw_rate'))
    beta_lo, beta_hi = _range(off.series('sideslip') + on.series('sideslip'))

    tmax = max(on.times())
    top = Plot(c, 78, 70, width - 40, 330, 0, tmax, yaw_lo, yaw_hi)
    top.frame()
    t_step, yaw_step = _nice_step(tmax, 8), _nice_step(yaw_hi - yaw_lo, 5)
    top.grid_at(t_step, yaw_step)
    _axis_labels(top, t_step, yaw_step, 1, 'TIME / S', 'DEG/S')
    top.plot(list(zip(off.times(), off.series('yaw_rate'))), C_OFF, 2)
    top.plot(list(zip(on.times(), on.series('yaw_rate'))), C_ON, 2)
    # 参考横摆角速度用细虚线压在两条实线之上：重合时看到虚线压在蓝线上，
    # 说明 ESC 把横摆角速度拉回了参考值
    top.plot(list(zip(on.times(), on.series('ref_yaw_rate'))), render.MUTED, 1, dashed=True)

    bottom = Plot(c, 78, 390, width - 40, 630, 0, tmax, beta_lo, beta_hi)
    bottom.frame()
    beta_step = _nice_step(beta_hi - beta_lo, 5)
    bottom.grid_at(t_step, beta_step)
    _axis_labels(bottom, t_step, beta_step, 1, 'TIME / S', 'DEG')
    bottom.plot(list(zip(off.times(), off.series('sideslip'))), C_OFF, 2)
    bottom.plot(list(zip(on.times(), on.series('sideslip'))), C_ON, 2)

    _legend(c, width - 330, 14, [('ESC OFF', C_OFF), ('ESC ON', C_ON)])
    c.text(width - 300, 352, 'TOP: YAW RATE / BOTTOM: SIDESLIP', C_MUTED)
    return c


# ---------------------------------------------------------------- 图 3 对开路面
def mu_split_figure(off: SimResult, on: SimResult, width: int = 1180,
                    height: int = 720) -> render.Canvas:
    """对开路面制动：四轮制动指令与横向偏移。"""
    c = render.Canvas(width, height)
    _title(c, f'{on.scenario.title_for_render}  ESC OFF VS ON',
           f'MU LEFT {on.scenario.road.mu_left:.2f} / RIGHT {on.scenario.road.mu_right:.2f}, '
           f'FULL BRAKE FROM {on.scenario.initial_speed * 3.6:.0f} KM/H')

    tmax = max(on.times())
    t_step = _nice_step(tmax, 8)

    top = Plot(c, 78, 78, width - 40, 330, 0, tmax, -0.05, 1.05)
    top.frame()
    top.grid_at(t_step, 0.25)
    _axis_labels(top, t_step, 0.25, 1, 'TIME / S', 'BRAKE CMD', digits=2)
    names = ('FL', 'FR', 'RL', 'RR')
    keys = ('brake_fl', 'brake_fr', 'brake_rl', 'brake_rr')
    for key, color in zip(keys, WHEEL_COLORS):
        top.plot(list(zip(on.times(), on.series(key))), color, 2)
    top.plot(list(zip(on.times(), [f.driver_brake for f in on.frames])), render.MUTED, 1,
             dashed=True)
    # 轮次图例放在坐标区上方靠中间，避开左侧的 BRAKE CMD 轴标题
    cursor = 200
    for name, color in zip(names, WHEEL_COLORS):
        c.line(cursor, 70, cursor + 12, 70, color, 3)
        cursor = c.text(cursor + 16, 66, name, render.MUTED) + 16

    drift = off.series('y') + on.series('y')
    d_lo, d_hi = _range(drift)
    d_step = _nice_step(d_hi - d_lo, 5)
    bottom = Plot(c, 78, 396, width - 40, 630, 0, tmax, d_lo, d_hi)
    bottom.frame()
    bottom.grid_at(t_step, d_step)
    _axis_labels(bottom, t_step, d_step, 1, 'TIME / S', 'LATERAL OFFSET / M')
    bottom.plot(list(zip(off.times(), off.series('y'))), C_OFF, 2)
    bottom.plot(list(zip(on.times(), on.series('y'))), C_ON, 2)
    _legend(c, width - 300, 14, [('ESC OFF', C_OFF), ('ESC ON', C_ON)])

    c.text(78, 352, 'TOP: WHEEL BRAKE COMMAND (ESC ON) / BOTTOM: LATERAL DRIFT', C_MUTED)
    c.text(width - 340, 352,
           f'STOP DISTANCE  OFF {off.metrics["stop_distance_m"]:.0f} m / '
           f'ON {on.metrics["stop_distance_m"]:.0f} m', C_MUTED)
    return c


# ---------------------------------------------------------------- 图 4 汇总
SHORT_NAMES = {
    'lane_change': 'LANE CHANGE',
    'sine_dwell': 'SINE DWELL',
    'mu_split': 'MU SPLIT',
    'wet_lane_change': 'WET LANE',
}


def summary_figure(pairs: list[tuple[SimResult, SimResult]], width: int = 1180,
                   height: int = 548) -> render.Canvas:
    """四个工况的最大侧偏角与横向偏移对比。"""
    c = render.Canvas(width, height)
    _title(c, 'SUMMARY  FOUR MANEUVERS', 'ORANGE: ESC OFF / BLUE: ESC ON')

    names = [SHORT_NAMES.get(off.scenario.name, off.scenario.name) for off, _ in pairs]
    beta_off = [off.metrics['max_beta_deg'] for off, _ in pairs]
    beta_on = [on.metrics['max_beta_deg'] for _, on in pairs]
    cross_off = [off.metrics['max_cross_track_m'] for off, _ in pairs]
    cross_on = [on.metrics['max_cross_track_m'] for _, on in pairs]

    def bars(rect, values_off, values_on, label, unit, digits):
        x0, y0, x1, y1 = rect
        ymax = _nice_step(max(values_off + values_on) * 1.15, 5)
        ymax = max(ymax, max(values_off + values_on) * 1.05)
        p = Plot(c, x0, y0, x1, y1, 0, len(names), 0, ymax)
        p.frame()
        step = _nice_step(ymax, 5)
        p.grid_at(1.0, step)
        y = 0.0
        while y <= ymax + 1e-6:
            px, py = p.px(0, y)
            c.line(px - 3, py, px, py, render.MUTED)
            c.text(px - c.text_width(f'{y:.0f}') - 6, py - 3, f'{y:.0f}', render.MUTED)
            y += step
        for index, (vo, vn) in enumerate(zip(values_off, values_on)):
            for offset, value, color in ((0.08, vo, C_OFF), (0.53, vn, C_ON)):
                bx0, by = p.px(index + offset, 0.0)
                bx1, _ = p.px(index + offset + 0.39, 0.0)
                _, ty = p.px(index, min(value, ymax))
                c.fill_rect(bx0 + 2, ty, bx1 - 2, by - 1, color)
                text = f'{value:.{digits}f}'
                c.text(bx0 + 3, ty - 11, text, render.MUTED)
            label_text = names[index]
            cx, _ = p.px(index + 0.5, 0)
            c.text(cx - c.text_width(label_text) / 2.0, y1 + 8, label_text, render.MUTED)
        c.text(x0, y0 - 26, label.upper() + ' / ' + unit, C_TEXT)

    bars((70, 92, width // 2 - 40, 505), beta_off, beta_on, 'max sideslip', 'deg', 1)
    bars((width // 2 + 10, 92, width - 40, 505), cross_off, cross_on,
         'max deviation', 'm', 1)
    return c


# ---------------------------------------------------------------- 动画
def animation_frames(off: SimResult, on: SimResult, width: int = 900,
                     max_frames: int = 70) -> list[render.Canvas]:
    """上下两栏俯视动画：上栏 ESC 关，下栏 ESC 开。

    每栏的高度按「等比例铺满」反推：纵向只放 ±HALF_SPAN 米，
    所以整张图画出来就是两条扁带子，不会上下留大片空白。
    """
    path: ReferencePath = on.scenario.path
    xmax = max(f.state.x for f in on.frames) * 1.02
    margin = 26
    half = 9.0
    row_h = int((width - 2 * margin) * 2.0 * half / xmax) + 2 * margin
    height = row_h * 2 + 4

    frames: list[render.Canvas] = []
    step = max(1, len(on.frames) // max_frames)
    for index in range(0, len(on.frames), step):
        c = render.Canvas(width, height)
        c.fill_rect(0, 0, width, height, render.BG)
        c.line(0, row_h + 2, width, row_h + 2, render.EDGE, 1)

        for row, (result, label, color) in enumerate(
                ((off, 'ESC OFF', C_OFF), (on, 'ESC ON', C_ON))):
            base_y = -row * 26.0
            view = render.View(0, xmax, base_y - half, base_y + half,
                               width, row_h, margin)
            view.off_y += row_h * row
            for edge in (-3.5, 3.5):
                c.line(*view.px(0, base_y + edge), *view.px(xmax, base_y + edge),
                       render.C_LANE, 1, dash=12)
            for (ax, ay), (bx, by) in zip(path.points, path.points[1:]):
                c.line(*view.px(ax, ay + base_y), *view.px(bx, by + base_y), C_REF, 2)

            frame = result.frames[min(index, len(result.frames) - 1)]
            trail = result.frames[:index + 1:3]
            for a, b in zip(trail, trail[1:]):
                c.line(*view.px(a.state.x, a.state.y + base_y),
                       *view.px(b.state.x, b.state.y + base_y), color, 2)

            px, py = view.px(frame.state.x, frame.state.y + base_y)
            draw_car(c, px, py, frame.state.yaw, color, 22.0, 10.0)
            c.text(14, 10 + row_h * row,
                   f'{label}  T={frame.t:4.1f} S  BETA={frame.state.sideslip * 57.3:5.1f} DEG',
                   color)
            if frame.esc.intensity > 0.02:
                c.text(width - 168, 10 + row_h * row,
                       f'ESC {frame.esc.intensity * 100:.0f}%', C_WARN)
        frames.append(c)
    return frames
