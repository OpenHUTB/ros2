"""用 Pillow 画 2×2 运行效果图，不依赖 matplotlib。

四张子图共用同一份 ``build_demo`` 结果：曲线、表格与结论来自同一次计算。
坐标轴、刻度、图例都是手工绘制的，因此只需要 numpy + Pillow。
"""

from __future__ import annotations

import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

CANVAS = (1600, 1150)
BACKGROUND = "#FFFFFF"
PANEL_FILL = "#F8FAFC"
PANEL_EDGE = "#CBD5E1"
GRID = "#E2E8F0"
TEXT = "#0F172A"
MUTED = "#64748B"
ALERT = "#B91C1C"
FADE_FILL = "#FEE2E2"
SPEED_COLORS = ("#1A73E8", "#E8710A", "#0F9D58")

_CN_FONTS = (
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)
_FONT_CACHE: dict = {}


def _font(size: int):
    if size not in _FONT_CACHE:
        path = next((p for p in _CN_FONTS if os.path.exists(p)), None)
        _FONT_CACHE[size] = (
            ImageFont.truetype(path, size) if path else ImageFont.load_default(size)
        )
    return _FONT_CACHE[size]


def _dashed(draw, start, end, fill, width=2, dash=9, gap=7) -> None:
    x0, y0 = start
    x1, y1 = end
    length = math.hypot(x1 - x0, y1 - y0)
    if length <= 0.0:
        return
    dx = (x1 - x0) / length
    dy = (y1 - y0) / length
    position = 0.0
    while position < length:
        stop = min(position + dash, length)
        draw.line(
            [(x0 + dx * position, y0 + dy * position), (x0 + dx * stop, y0 + dy * stop)],
            fill=fill,
            width=width,
        )
        position = stop + gap


def _fmt(value: float, digits: int = 0) -> str:
    return f"{float(value):.{digits}f}"


def _ticks(low: float, high: float, count: int):
    return [low + (high - low) * index / (count - 1) for index in range(count)]


def _nice_step(span: float) -> float:
    raw = span / 5.0
    magnitude = 10.0 ** math.floor(math.log10(raw))
    for factor in (1.0, 2.0, 2.5, 5.0, 10.0):
        if raw <= factor * magnitude:
            return factor * magnitude
    return 10.0 * magnitude


class _Panel:
    def __init__(self, draw, box, xlim, ylim, title, xlabel, ylabel,
                 xdigits=0, ydigits=1, xticks=None, yticks=None) -> None:
        self.draw = draw
        self.left, self.top, self.right, self.bottom = box
        self.xlim = (float(xlim[0]), float(xlim[1]))
        self.ylim = (float(ylim[0]), float(ylim[1]))
        self.xdigits = xdigits
        self.ydigits = ydigits
        draw.rectangle(box, fill=PANEL_FILL, outline=PANEL_EDGE, width=2)
        draw.text((self.left, self.top - 50), title, fill=TEXT, font=_font(20))
        draw.text((self.left, self.top - 27), ylabel, fill=MUTED, font=_font(15))
        width = draw.textlength(xlabel, font=_font(15))
        draw.text(((self.left + self.right - width) / 2.0, self.bottom + 32),
                  xlabel, fill=MUTED, font=_font(15))
        if xticks is None:
            step = _nice_step(self.xlim[1] - self.xlim[0])
            xticks = []
            value = math.ceil(self.xlim[0] / step) * step
            while value <= self.xlim[1] + 1e-9:
                xticks.append(value)
                value += step
        if yticks is None:
            step = _nice_step(self.ylim[1] - self.ylim[0])
            yticks = []
            value = math.ceil(self.ylim[0] / step) * step
            while value <= self.ylim[1] + 1e-9:
                yticks.append(value)
                value += step
        self._grid(xticks, yticks)

    def px(self, x: float) -> float:
        low, high = self.xlim
        return self.left + (float(x) - low) / (high - low) * (self.right - self.left)

    def py(self, y: float) -> float:
        low, high = self.ylim
        return self.bottom - (float(y) - low) / (high - low) * (self.bottom - self.top)

    def _grid(self, xticks, yticks) -> None:
        for value in xticks:
            if not self.xlim[0] <= value <= self.xlim[1]:
                continue
            x = self.px(value)
            self.draw.line([(x, self.top), (x, self.bottom)], fill=GRID, width=1)
            label = _fmt(value, self.xdigits)
            width = self.draw.textlength(label, font=_font(14))
            self.draw.text((x - width / 2.0, self.bottom + 8), label, fill=MUTED, font=_font(14))
        for value in yticks:
            if not self.ylim[0] <= value <= self.ylim[1]:
                continue
            y = self.py(value)
            self.draw.line([(self.left, y), (self.right, y)], fill=GRID, width=1)
            label = _fmt(value, self.ydigits)
            width = self.draw.textlength(label, font=_font(14))
            self.draw.text((self.left - 9 - width, y - 8), label, fill=MUTED, font=_font(14))

    def line(self, xs, ys, color, width=3) -> None:
        points = []
        for x, y in zip(np.asarray(xs, float), np.asarray(ys, float)):
            if not (self.xlim[0] <= x <= self.xlim[1] and self.ylim[0] <= y <= self.ylim[1]):
                continue
            points.append((self.px(x), self.py(y)))
        if len(points) > 1:
            self.draw.line(points, fill=color, width=width, joint="curve")

    def marker(self, x: float, y: float, color, radius=6) -> None:
        cx, cy = self.px(x), self.py(y)
        self.draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                          fill=color, outline="white", width=2)

    def bar(self, x: float, y: float, width_px: float, color) -> None:
        base = self.py(self.ylim[0])
        top = self.py(y)
        self.draw.rectangle([self.px(x) - width_px / 2, top,
                             self.px(x) + width_px / 2, base], fill=color)

    def hline(self, y, color, width=2, dashed=True) -> None:
        start, end = (self.left, self.py(y)), (self.right, self.py(y))
        if dashed:
            _dashed(self.draw, start, end, color, width=width)
        else:
            self.draw.line([start, end], fill=color, width=width)

    def vline(self, x, color, width=2, dashed=True) -> None:
        start, end = (self.px(x), self.top), (self.px(x), self.bottom)
        if dashed:
            _dashed(self.draw, start, end, color, width=width)
        else:
            self.draw.line([start, end], fill=color, width=width)

    def shade(self, x0, x1, fill) -> None:
        self.draw.rectangle([self.px(x0), self.top, self.px(x1), self.bottom], fill=fill)


def _legend(draw, origin, entries) -> None:
    x, y = origin
    for color, label in entries:
        draw.line([(x, y + 10), (x + 28, y + 10)], fill=color, width=4)
        draw.text((x + 36, y), label, fill=TEXT, font=_font(16))
        x += 44 + draw.textlength(label, font=_font(16))


def render(bundle, path: str) -> str:
    """把 ``build_demo`` 的结果画成 PNG 并写到 ``path``。"""
    image = Image.new("RGB", CANVAS, BACKGROUND)
    draw = ImageDraw.Draw(image)

    disc, braking = bundle.disc, bundle.braking
    curve = disc.curve
    descents = bundle.descents
    cycles = bundle.cycles

    draw.text((40, 22), "制动器温升与热衰退评估", fill=TEXT, font=_font(32))
    subtitle = (
        f"单盘热容 {disc.spec.heat_capacity_j_per_k / 1000:.1f} kJ/K，"
        f"散热 {disc.spec.cooling_w_per_k:.0f} W/K，"
        f"摩擦系数 {curve.mu_cold:.2f} → {curve.mu_hot:.2f}"
        f"（{curve.fade_start_c:.0f}–{curve.fade_end_c:.0f} °C 内衰退）；"
        f"整车 {braking.mass_kg:.0f} kg，常温减速度 {braking.reference_decel_mps2:.1f} m/s²"
    )
    draw.text((40, 70), subtitle, fill=MUTED, font=_font(17))
    draw.line([(40, 104), (CANVAS[0] - 40, 104)], fill=PANEL_EDGE, width=2)

    # ------------------------------------------------------ (a) 长下坡温度时程
    top_temp = max(float(run.temperature_c.max()) for run in descents)
    duration_s = float(descents[0].time_s[-1])
    panel_a = _Panel(
        draw, (120, 200, 780, 560),
        xlim=(0.0, duration_s),
        ylim=(0.0, math.ceil((top_temp + 40.0) / 100.0) * 100.0),
        title="(a) 恒速长下坡：制动盘温度",
        xlabel="时间 t / s",
        ylabel="盘温 / °C",
        ydigits=0,
    )
    panel_a.hline(curve.fade_start_c, ALERT)
    draw.text((panel_a.right - 150, panel_a.py(curve.fade_start_c) - 22),
              f"衰退起始 {curve.fade_start_c:.0f} °C", fill=ALERT, font=_font(14))
    for color, run in zip(SPEED_COLORS, descents):
        panel_a.line(run.time_s, run.temperature_c, color)
        if run.fade_onset_s is not None:
            panel_a.marker(run.fade_onset_s, curve.fade_start_c, color, radius=5)
    _legend(draw, (120, 612),
            [(color, f"{run.speed_kmh:.0f} km/h") for color, run in zip(SPEED_COLORS, descents)])

    # --------------------------------------------------- (b) 摩擦系数衰退曲线
    panel_b = _Panel(
        draw, (900, 200, 1560, 560),
        xlim=(0.0, 700.0),
        ylim=(0.15, 0.45),
        title="(b) 摩擦系数随盘温衰退与三个工况平衡点",
        xlabel="盘温 / °C",
        ylabel="摩擦系数 μ",
        ydigits=2,
    )
    panel_b.shade(curve.fade_start_c, min(curve.fade_end_c, 700.0), FADE_FILL)
    temperatures = np.linspace(0.0, 700.0, 351)
    panel_b.line(temperatures, curve.mu(temperatures), "#334155", width=3)
    for color, run in zip(SPEED_COLORS, descents):
        if run.equilibrium_c is not None:
            panel_b.marker(run.equilibrium_c, float(curve.mu(run.equilibrium_c)), color, radius=7)
    draw.text((panel_b.left + 10, panel_b.top + 10), "浅红：衰退区间", fill=ALERT, font=_font(14))
    note = "、".join(
        f"{run.speed_kmh:.0f} km/h→{run.equilibrium_c:.0f} °C"
        for run in descents if run.equilibrium_c is not None
    )
    draw.text((900, 612), "平衡温度：" + note, fill=MUTED, font=_font(15))

    # ------------------------------------------------------ (c) 连续制动温升
    max_cycle_temp = float(cycles.temperature_after_c.max())
    panel_c = _Panel(
        draw, (120, 730, 780, 1030),
        xlim=(0.5, float(cycles.cycle.size) + 0.5),
        ylim=(0.0, math.ceil((max_cycle_temp + 30.0) / 100.0) * 100.0),
        title=f"(c) 连续 {cycles.cycle.size} 次急刹（{cycles.speed_kmh:.0f}→0 km/h，间隔 "
              f"{cycles.cooldown_s:.0f} s）",
        xlabel="制动次序",
        ylabel="盘温 / °C",
        ydigits=0,
        xticks=list(range(1, int(cycles.cycle.size) + 1, 3)),
    )
    panel_c.hline(curve.fade_start_c, ALERT)
    for index, (before, after) in enumerate(
        zip(cycles.temperature_before_c, cycles.temperature_after_c), start=1
    ):
        color = ALERT if before >= curve.fade_start_c else "#94A3B8"
        panel_c.bar(index, after, 26, color)
    panel_c.line(cycles.cycle, cycles.temperature_before_c, "#1A73E8", width=2)
    _legend(draw, (panel_c.left + 14, panel_c.top + 12),
            [(ALERT, "本次已进入衰退区"), ("#94A3B8", "本次未进入衰退区")])

    # ------------------------------------------------------------ (d) 指标表
    table = (900, 730, 1560, 1060)
    draw.rectangle(table, fill=PANEL_FILL, outline=PANEL_EDGE, width=2)
    draw.text((table[0], table[1] - 50), "(d) 指标汇总", fill=TEXT, font=_font(20))
    columns = (("车速", 16, 70), ("单盘功率", 96, 90), ("平衡温度", 196, 90),
               ("进入衰退", 296, 90), ("停车距离", 396, 96), ("相对冷态", 500, 96))
    header_y = table[1] + 16
    for name, offset, width in columns:
        text_width = draw.textlength(name, font=_font(14))
        draw.text((table[0] + offset + width - text_width, header_y), name,
                  fill=MUTED, font=_font(14))
    draw.line([(table[0] + 12, header_y + 26), (table[2] - 12, header_y + 26)],
              fill=PANEL_EDGE, width=2)
    draw.text((table[0] + 16, header_y + 36), "恒速长下坡（7% 坡度，600 s）", fill=TEXT, font=_font(15))
    for index, run in enumerate(descents):
        y = header_y + 66 + index * 30
        cold = run.stop_distance_cold_m
        growth = (run.stop_distance_m / cold - 1.0) * 100.0
        values = (
            f"{run.speed_kmh:.0f} km/h",
            f"{run.power_per_disc_w / 1000.0:.1f} kW",
            "—" if run.equilibrium_c is None else f"{run.equilibrium_c:.0f} °C",
            "未衰退" if run.fade_onset_s is None else f"{run.fade_onset_s:.0f} s",
            f"{run.stop_distance_m:.1f} m",
            f"{growth:+.1f}%",
        )
        for (name, offset, width), value in zip(columns, values):
            text_width = draw.textlength(value, font=_font(14))
            colour = ALERT if (name == "相对冷态" and growth > 1.0) or (
                name == "进入衰退" and run.fade_onset_s is not None) else TEXT
            draw.text((table[0] + offset + width - text_width, y), value,
                      fill=colour, font=_font(14))

    y = header_y + 66 + len(descents) * 30 + 14
    first = float(cycles.distance_m[0])
    last = float(cycles.distance_m[-1])
    lines = [
        f"连续 {cycles.cycle.size} 次急刹：最高盘温 {max_cycle_temp:.0f} °C，"
        f"停车距离 {first:.1f} → {last:.1f} m（{(last / first - 1.0) * 100.0:+.1f}%，"
        f"冷态 {cycles.distance_cold_m:.1f} m）",
        f"长下坡 1 min 与 10 min 的对比："
        + "；".join(
            f"{run.speed_kmh:.0f} km/h {float(run.temperature_c[len(run.time_s) // 150]):.0f}→"
            f"{float(run.temperature_c[-1]):.0f} °C"
            for run in descents
        ),
        "说明：单次制动按起始盘温计算距离；温度累积由热模型逐次积分。",
    ]
    for offset, text in enumerate(lines):
        draw.text((table[0] + 16, y + offset * 26), text,
                  fill=MUTED if offset == len(lines) - 1 else TEXT, font=_font(14))

    footer = (
        "数据来源：python main.py demo（集总热容 + 分段线性衰退，0.5 s / 0.05 s 定步长 RK4；"
        "未含盘内温度梯度、刹车片磨损与对流随车速变化；定义见 docs/ground/brake_thermal_fade.md）"
    )
    draw.text((40, CANVAS[1] - 42), footer, fill=MUTED, font=_font(14))

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    image.save(path, format="PNG", optimize=True)
    return path
