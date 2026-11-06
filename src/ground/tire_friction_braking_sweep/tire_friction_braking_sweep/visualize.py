"""用 Pillow 画 2×2 运行效果图，不依赖 matplotlib。"""

from __future__ import annotations

import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .model import GRAVITY, braking_distance_m, saturated_decel_mps2

CANVAS = (1600, 1150)
BACKGROUND = "#FFFFFF"
PANEL_FILL = "#F8FAFC"
PANEL_EDGE = "#CBD5E1"
GRID = "#E2E8F0"
TEXT = "#0F172A"
MUTED = "#64748B"
ALERT = "#B91C1C"
MEASURED = "#1A73E8"
THEORY = "#0F9D58"
FIT = "#E8710A"
FADE_FILL = "#FEF3C7"

_CN_FONTS = (
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
)
_CACHE: dict = {}


def _font(size: int):
    if size not in _CACHE:
        path = next((p for p in _CN_FONTS if os.path.exists(p)), None)
        _CACHE[size] = (
            ImageFont.truetype(path, size) if path else ImageFont.load_default(size)
        )
    return _CACHE[size]


def _dashed(draw, start, end, fill, width=2, dash=9, gap=7) -> None:
    x0, y0 = start
    x1, y1 = end
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 0.0:
        return
    dx, dy = (x1 - x0) / length, (y1 - y0) / length
    position = 0.0
    while position < length:
        stop = min(position + dash, length)
        draw.line([(x0 + dx * position, y0 + dy * position),
                   (x0 + dx * stop, y0 + dy * stop)], fill=fill, width=width)
        position = stop + gap


def _fmt(value: float, digits: int = 2) -> str:
    return f"{float(value):.{digits}f}"


def _ticks(low: float, high: float, count: int):
    return [low + (high - low) * index / (count - 1) for index in range(count)]


class _Panel:
    def __init__(self, draw, box, xlim, ylim, title, xlabel, ylabel,
                 xdigits=1, ydigits=1, xticks=None, yticks=None) -> None:
        self.draw = draw
        self.left, self.top, self.right, self.bottom = box
        self.xlim, self.ylim = (float(xlim[0]), float(xlim[1])), (float(ylim[0]), float(ylim[1]))
        self.xdigits, self.ydigits = xdigits, ydigits
        draw.rectangle(box, fill=PANEL_FILL, outline=PANEL_EDGE, width=2)
        draw.text((self.left, self.top - 50), title, fill=TEXT, font=_font(19))
        draw.text((self.left, self.top - 26), ylabel, fill=MUTED, font=_font(14))
        width = draw.textlength(xlabel, font=_font(14))
        draw.text(((self.left + self.right - width) / 2.0, self.bottom + 30),
                  xlabel, fill=MUTED, font=_font(14))
        self._grid(xticks or _ticks(*self.xlim, 6), yticks or _ticks(*self.ylim, 5))

    def px(self, x):
        return self.left + (float(x) - self.xlim[0]) / (self.xlim[1] - self.xlim[0]) * (self.right - self.left)

    def py(self, y):
        return self.bottom - (float(y) - self.ylim[0]) / (self.ylim[1] - self.ylim[0]) * (self.bottom - self.top)

    def _grid(self, xticks, yticks) -> None:
        for value in xticks:
            if not self.xlim[0] <= value <= self.xlim[1]:
                continue
            x = self.px(value)
            self.draw.line([(x, self.top), (x, self.bottom)], fill=GRID, width=1)
            label = _fmt(value, self.xdigits)
            self.draw.text((x - self.draw.textlength(label, font=_font(13)) / 2, self.bottom + 6),
                           label, fill=MUTED, font=_font(13))
        for value in yticks:
            if not self.ylim[0] <= value <= self.ylim[1]:
                continue
            y = self.py(value)
            self.draw.line([(self.left, y), (self.right, y)], fill=GRID, width=1)
            label = _fmt(value, self.ydigits)
            self.draw.text((self.left - 8 - self.draw.textlength(label, font=_font(13)), y - 7),
                           label, fill=MUTED, font=_font(13))

    def line(self, xs, ys, color, width=3, dashed=False) -> None:
        points = []
        for x, y in zip(np.asarray(xs, float), np.asarray(ys, float)):
            if not (self.xlim[0] <= x <= self.xlim[1] and self.ylim[0] <= y <= self.ylim[1]):
                continue
            points.append((self.px(x), self.py(y)))
        for p0, p1 in zip(points, points[1:]):
            if dashed:
                _dashed(self.draw, p0, p1, color, width=width)
            else:
                self.draw.line([p0, p1], fill=color, width=width)

    def markers(self, xs, ys, color, radius=6) -> None:
        for x, y in zip(xs, ys):
            cx, cy = self.px(x), self.py(y)
            self.draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                              fill=color, outline="white", width=2)

    def hline(self, y, color, width=2, dashed=True) -> None:
        start, end = (self.left, self.py(y)), (self.right, self.py(y))
        if dashed:
            _dashed(self.draw, start, end, color, width=width)
        else:
            self.draw.line([start, end], fill=color, width=width)

    def vbar(self, x, value, color, width_px=26) -> None:
        base = self.py(0.0)
        top = self.py(value)
        self.draw.rectangle([self.px(x) - width_px / 2, min(base, top),
                             self.px(x) + width_px / 2, max(base, top)], fill=color)


def _legend(draw, origin, entries) -> None:
    x, y = origin
    for color, label, dashed in entries:
        if dashed:
            _dashed(draw, (x, y + 10), (x + 28, y + 10), color, width=3)
        else:
            draw.line([(x, y + 10), (x + 28, y + 10)], fill=color, width=4)
        draw.text((x + 36, y), label, fill=TEXT, font=_font(15))
        x += 44 + draw.textlength(label, font=_font(15))


def render(dataset, summary: dict, path: str) -> str:
    """把扫掠数据与摘要画成 PNG。"""
    image = Image.new("RGB", CANVAS, BACKGROUND)
    draw = ImageDraw.Draw(image)

    points = summary["points"]
    fit = summary["fit"]
    mus = np.array([p["tire_friction"] for p in points])
    distances = np.array([p["distance_m"] for p in points])
    decels = np.array([p["decel_mps2"] for p in points])
    ratios = np.array([p["distance_ratio"] for p in points])
    speed = float(points[0]["entry_speed_kmh"]) / 3.6
    mu_max = float(mus.max())
    source = "模拟器实测" if summary.get("source") == "carla" else "离线示例数据"
    env = summary.get("environment") or {}

    draw.text((40, 20), "轮胎附着系数与制动距离：瓶颈在哪一侧", fill=TEXT, font=_font(31))
    subtitle = (
        f"{source}｜{env.get('map', '—')}／{env.get('vehicle', '—')}｜"
        f"{_fmt(points[0]['entry_speed_kmh'], 1)} km/h 全力制动｜"
        f"μ {_fmt(mus.min(), 2)}–{_fmt(mu_max, 2)}"
    )
    draw.text((40, 66), subtitle, fill=MUTED, font=_font(16))
    draw.line([(40, 100), (CANVAS[0] - 40, 100)], fill=PANEL_EDGE, width=2)

    # ------------------------------------------------- (a) 制动距离 vs μ
    panel_a = _Panel(draw, (120, 200, 780, 560), xlim=(0.0, mu_max * 1.05),
                     ylim=(0.0, math.ceil(distances.max() / 10.0) * 10.0),
                     title="(a) 制动距离随附着系数变化",
                     xlabel="轮胎附着系数 μ", ylabel="制动距离 / m",
                     xdigits=1, ydigits=0)
    grid = np.linspace(0.05, mu_max * 1.05, 200)
    panel_a.line(grid, braking_distance_m(speed, grid * GRAVITY), THEORY, width=2, dashed=True)
    panel_a.line(grid, braking_distance_m(speed, saturated_decel_mps2(grid, fit["a_max_mps2"], fit["mu_half"])),
                 FIT, width=2)
    panel_a.line(mus, distances, MEASURED, width=3)
    panel_a.markers(mus, distances, MEASURED)
    _legend(draw, (120, 606), [(MEASURED, "实测", False), (THEORY, "纯附着上限 v²/(2μg)", True),
                               (FIT, "饱和拟合", False)])

    # ------------------------------------------------- (b) 减速度 vs μ
    panel_b = _Panel(draw, (880, 200, 1540, 560), xlim=(0.0, mu_max * 1.05),
                     ylim=(0.0, math.ceil(mus.max() * GRAVITY / 2.0) * 2.0),
                     title="(b) 等效减速度与两个上限",
                     xlabel="轮胎附着系数 μ", ylabel="减速度 / (m/s²)",
                     xdigits=1, ydigits=0)
    panel_b.line(grid, grid * GRAVITY, THEORY, width=2, dashed=True)
    panel_b.hline(fit["a_max_mps2"], FIT, width=2)
    panel_b.line(grid, saturated_decel_mps2(grid, fit["a_max_mps2"], fit["mu_half"]), FIT, width=2)
    panel_b.line(mus, decels, MEASURED, width=3)
    panel_b.markers(mus, decels, MEASURED)
    draw.text((panel_b.left + 10, panel_b.top + 8), "灰绿虚线：μ g（附着上限）", fill=THEORY, font=_font(13))
    draw.text((panel_b.left + 10, panel_b.top + 52),
              f"橙色虚线：a_max 外推 {_fmt(fit['a_max_mps2'], 1)}（仅供参考）", fill=FIT, font=_font(13))
    draw.text((880, 606), f"距离比（实测/附着上限）{_fmt(ratios.min(), 2)}–{_fmt(ratios.max(), 2)} 倍",
              fill=MUTED, font=_font(14))

    # --------------------------------------------- (c) 距离弹性
    elasticities = [p.get("elasticity") for p in points]
    pair_mus = [(mus[i] + mus[i + 1]) / 2 for i in range(len(mus) - 1)]
    pair_values = [elasticities[i + 1] if elasticities[i + 1] is not None else 0.0
                   for i in range(len(mus) - 1)]
    low = min(-1.0, math.floor(min(pair_values) * 10) / 10)
    panel_c = _Panel(draw, (120, 700, 780, 1040), xlim=(0.0, mu_max * 1.05),
                     ylim=(low, 0.0), title="(c) 距离弹性 (Δd/d)/(Δμ/μ)",
                     xlabel="相邻两点的中点附着系数", ylabel="弹性（负值越小＝越敏感）",
                     xdigits=1, ydigits=1)
    for mu_value, value in zip(pair_mus, pair_values):
        color = ALERT if abs(value) < 0.20 else MEASURED
        panel_c.vbar(mu_value, value, color, width_px=30)
    panel_c.hline(-0.20, FIT, width=2)
    draw.text((panel_c.right - 210, panel_c.py(-0.20) + 6), "判定阈值 |弹性| < 0.2", fill=FIT, font=_font(13))

    # --------------------------------------------------- (d) 表格
    table = (880, 700, 1540, 1040)
    draw.rectangle(table, fill=PANEL_FILL, outline=PANEL_EDGE, width=2)
    draw.text((table[0], table[1] - 46), "(d) 每个附着系数的实测与判定", fill=TEXT, font=_font(19))
    columns = (("μ", 16, 46), ("距离 m", 74, 80), ("a m/s²", 168, 74),
               ("a/μg", 254, 66), ("弹性", 330, 66), ("瓶颈", 408, 120))
    header_y = table[1] + 14
    for name, offset, width in columns:
        text_width = draw.textlength(name, font=_font(13))
        draw.text((table[0] + offset + width - text_width, header_y), name, fill=MUTED, font=_font(13))
    draw.line([(table[0] + 12, header_y + 24), (table[2] - 12, header_y + 24)], fill=PANEL_EDGE, width=2)
    for index, point in enumerate(points):
        y = header_y + 34 + index * 32
        elastic = point.get("elasticity")
        values = (
            _fmt(point["tire_friction"], 2),
            _fmt(point["distance_m"], 2),
            _fmt(point["decel_mps2"], 2),
            _fmt(point["distance_ratio"], 2),
            "—" if elastic is None else _fmt(elastic, 3),
            point["bottleneck"],
        )
        for (name, offset, width), value in zip(columns, values):
            text_width = draw.textlength(value, font=_font(14))
            colour = ALERT if (name == "瓶颈" and value == "制动器受限") else TEXT
            draw.text((table[0] + offset + width - text_width, y), value, fill=colour, font=_font(14))

    note_y = header_y + 34 + len(points) * 32 + 10
    speed_range = summary.get("range", {})
    lines = [
        f"μ 从 {_fmt(speed_range.get('mu_min', mus.min()), 2)} 提到 "
        f"{_fmt(speed_range.get('mu_max', mu_max), 2)}，制动距离缩短 "
        f"{_fmt(speed_range.get('distance_reduction_percent', 0.0), 1)}%",
        f"外推极限 a_max={_fmt(fit['a_max_mps2'], 2)} m/s²、μ½={_fmt(fit['mu_half'], 2)}，"
        f"R²={_fmt(fit['r2'], 3)}；该极限超出实测范围，仅用于描述饱和趋势",
    ]
    for index, text in enumerate(lines):
        draw.text((table[0] + 14, note_y + index * 30), text,
                  fill=MUTED if index else TEXT, font=_font(14))

    footer = (
        "数据来源：python main.py sim（HUTB 模拟器，同步模式 0.05 s 步长，改 tire_friction 后全力制动）；"
        "定义见 docs/ground/tire_friction_braking_sweep.md"
    )
    draw.text((40, CANVAS[1] - 40), footer, fill=MUTED, font=_font(13))

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    image.save(path, format="PNG", optimize=True)
    return path
