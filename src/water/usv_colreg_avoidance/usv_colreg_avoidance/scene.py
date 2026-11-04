"""把仿真结果画成俯视海图：轨迹、船位、安全圈、会遇标注。

同一套绘制函数同时服务于 GIF 与 PNG，只换画布尺寸。
"""
from __future__ import annotations

import math

from . import render
from .render import Canvas, View
from .sim import SimResult

TITLE_H = 40
LEGEND_H = 26


def _names(result: SimResult) -> str:
    return '   '.join(f'{"ABC"[i]}={n}' for i, n in enumerate(result.goal_names))


def _nice_step(span: float) -> float:
    for step in (250, 500, 1000, 2000, 5000):
        if span / step <= 8:
            return step
    return 10000


def bounds(result: SimResult, pad_ratio: float = 0.06) -> tuple[float, float, float, float]:
    xs = [x for f in result.frames for (x, _, _) in f.ships]
    ys = [y for f in result.frames for (_, y, _) in f.ships]
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    pad_x = (xmax - xmin) * pad_ratio + 150.0
    pad_y = (ymax - ymin) * pad_ratio + 150.0
    return xmin - pad_x, xmax + pad_x, ymin - pad_y, ymax + pad_y


def _draw_grid(canvas: Canvas, view: View, box: tuple[float, float, float, float],
               top: int, bottom: int) -> None:
    xmin, xmax, ymin, ymax = box
    step = _nice_step(max(xmax - xmin, ymax - ymin))
    x = math.floor(xmin / step) * step
    while x <= xmax:
        px, _ = view.px(x, ymin)
        canvas.line(px, top, px, bottom, render.GRID, 1)
        x += step
    y = math.floor(ymin / step) * step
    while y <= ymax:
        _, py = view.px(xmin, y)
        canvas.line(0, py, canvas.w, py, render.GRID, 1)
        y += step


def _ship_polygon(view: View, x: float, y: float, heading: float,
                  size: float = 13.0) -> list[tuple[float, float]]:
    px, py = view.px(x, y)
    h = math.radians(heading)
    fx, fy = math.sin(h), -math.cos(h)
    sx, sy = -fy, fx
    return [
        (px + fx * size, py + fy * size),
        (px - fx * size * 0.62 + sx * size * 0.52, py - fy * size * 0.62 + sy * size * 0.52),
        (px - fx * size * 0.30, py - fy * size * 0.30),
        (px - fx * size * 0.62 - sx * size * 0.52, py - fy * size * 0.62 - sy * size * 0.52),
    ]


def draw(result: SimResult, upto: int, canvas: Canvas, box: tuple[float, float, float, float],
         title: str, legend: str, time_text: str, d_safe: float,
         mark_min_sep: bool = False, scale_factor: float = 1.0) -> None:
    view = View(*box, canvas.w, canvas.h, margin=int(24 * scale_factor))
    top = TITLE_H * scale_factor
    bottom = canvas.h - LEGEND_H * scale_factor

    _draw_grid(canvas, view, box, top, bottom)

    # 轨迹：按固定步长抽样，但用线段相连，保证曲线连续
    stride = max(1, len(result.frames) // 400)
    for i in range(1, min(upto + 1, len(result.frames))):
        if i % stride:
            continue
        prev = result.frames[i - 1]
        cur = result.frames[i]
        for k in range(len(cur.ships)):
            x0, y0 = view.px(prev.ships[k][0], prev.ships[k][1])
            x1, y1 = view.px(cur.ships[k][0], cur.ships[k][1])
            width = 2 if k == 0 else 1
            canvas.line(x0, y0, x1, y1, render.SHIP_COLORS[k % len(render.SHIP_COLORS)], width)

    # 起点
    for k, ship in enumerate(result.initial):
        px, py = view.px(ship.x, ship.y)
        canvas.fill_rect(px - 2 * scale_factor, py - 2 * scale_factor,
                         px + 2 * scale_factor, py + 2 * scale_factor,
                         render.SHIP_COLORS[k % len(render.SHIP_COLORS)])

    # 安全圈（只画正在避让的船，避免图面太乱）
    for i, (x, y, _) in enumerate(result.frames[upto].ships):
        if d_safe > 0 and i == 0:
            px, py = view.px(x, y)
            canvas.ring(px, py, view.length(d_safe), render.SAFE_RING, 1, dash_deg=12.0)

    # 最小会遇距离处的连线
    if mark_min_sep and not math.isinf(result.metrics['min_separation']):
        t_min = result.metrics['min_separation_time']
        idx = min(range(len(result.frames)),
                  key=lambda i: abs(result.frames[i].t - t_min))
        frame = result.frames[idx]
        points = []
        for (x, y, _) in frame.ships[:2]:
            points.append(view.px(x, y))
        if len(points) == 2:
            canvas.line(*points[0], *points[1], render.WARN, 1, dash=4)
            mx = (points[0][0] + points[1][0]) / 2
            my = (points[0][1] + points[1][1]) / 2
            label = f'{result.metrics["min_separation"]:.0f} M'
            canvas.fill_rect(int(mx) - 2, int(my) - 22 * scale_factor,
                             int(mx) + canvas.text_width(label, int(scale_factor)) + 2,
                             int(my) - 22 * scale_factor + 12 * scale_factor, render.WHITE)
            canvas.text(int(mx) + 1, int(my) - 20 * scale_factor, label, render.WARN,
                    max(1, int(1.4 * scale_factor)))

    # 船体
    for i, (x, y, heading) in enumerate(result.frames[upto].ships):
        color = render.SHIP_COLORS[i % len(render.SHIP_COLORS)]
        canvas.poly(_ship_polygon(view, x, y, heading, 12.0 * scale_factor), color)
        canvas.poly(_ship_polygon(view, x, y, heading, 7.0 * scale_factor), render.WHITE)
        px, py = view.px(x, y)
        canvas.text(int(px) - 3 * int(scale_factor), int(py) - 3 * int(scale_factor),
                    'ABC'[i % 3], render.HULL, int(scale_factor))

    # 标题栏
    canvas.fill_rect(0, 0, canvas.w, int(top) - 1, render.WHITE)
    canvas.line(0, int(top) - 1, canvas.w, int(top) - 1, render.EDGE, 1)
    canvas.text(int(10 * scale_factor), int(4 * scale_factor), title, render.TEXT,
                int(2 * scale_factor))
    canvas.text(int(10 * scale_factor), int(25 * scale_factor), legend, render.MUTED,
                max(1, int(1.5 * scale_factor)))
    time_scale = max(1, int(1.5 * scale_factor))
    tw = canvas.text_width(time_text, time_scale)
    canvas.text(canvas.w - tw - int(10 * scale_factor), int(6 * scale_factor),
                time_text, render.TEXT, time_scale)

    # 底部刻度尺
    bar_m = _nice_step(max(box[1] - box[0], box[3] - box[2]))
    while view.length(bar_m) > canvas.w / 3.0 and bar_m > 100:
        bar_m /= 2
    bar_px = view.length(bar_m)
    bx = int(10 * scale_factor)
    by = canvas.h - int(10 * scale_factor)
    canvas.fill_rect(0, canvas.h - int(LEGEND_H * scale_factor), canvas.w, canvas.h, render.WHITE)
    canvas.line(bx, by, bx + bar_px, by, render.TEXT, 2)
    canvas.line(bx, by - 3, bx, by + 3, render.TEXT, 1)
    canvas.line(bx + bar_px, by - 3, bx + bar_px, by + 3, render.TEXT, 1)
    canvas.text(int(bx + bar_px + 6 * scale_factor), int(by - 4 * scale_factor),
                f'{bar_m:.0f} M', render.MUTED, max(1, int(1.4 * scale_factor)))

    # 图例：这条船到现在为止扮演过什么角色
    seen = [set() for _ in result.initial]
    for frame in result.frames[:upto + 1]:
        for i, role in enumerate(frame.roles):
            seen[i].add(role)

    lx = canvas.w - int(150 * scale_factor)
    for i in range(min(3, len(result.initial))):
        ly = canvas.h - int(LEGEND_H * scale_factor) + int(4 * scale_factor)
        color = render.SHIP_COLORS[i % len(render.SHIP_COLORS)]
        canvas.fill_rect(lx + i * int(48 * scale_factor), ly,
                         lx + i * int(48 * scale_factor) + int(6 * scale_factor),
                         ly + int(6 * scale_factor), color)
        if 'give-way' in seen[i]:
            tag = 'GW'
        elif 'stand-on' in seen[i]:
            tag = 'SO'
        else:
            tag = '--'
        canvas.text(lx + i * int(50 * scale_factor) + int(8 * scale_factor), ly,
                    'ABC'[i] + ' ' + tag, render.TEXT, max(1, int(1.4 * scale_factor)))


def render_still(result: SimResult, width: int = 960, height: int = 720,
                 d_safe: float = 400.0) -> Canvas:
    """一张静态总览图：完整轨迹 + 最小会遇距离标注。"""
    scale = width / 640.0
    canvas = Canvas(width, height)
    box = bounds(result)
    time_text = f'T = {result.frames[-1].t:.0f} S'
    draw(result, len(result.frames) - 1, canvas, box, result.title_en, _names(result),
         time_text, d_safe, mark_min_sep=True, scale_factor=scale)
    return canvas


def render_animation(result: SimResult, frames: int = 72, width: int = 640, height: int = 480,
                     d_safe: float = 400.0) -> list[Canvas]:
    """按等间隔抽帧，画一套动画帧。"""
    total = len(result.frames)
    picks = [int(round(i * (total - 1) / (frames - 1))) for i in range(frames)]
    box = bounds(result)
    out = []
    for idx in picks:
        canvas = Canvas(width, height)
        time_text = f'T = {result.frames[idx].t:.0f} S'
        draw(result, idx, canvas, box, result.title_en, _names(result),
             time_text, d_safe)
        out.append(canvas)
    return out
