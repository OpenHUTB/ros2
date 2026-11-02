"""用 Pillow 画 2×2 运行效果图，不依赖 matplotlib。

四张子图共用一份 ``build_demo`` 结果，所以图上的曲线、表格里的指标和标注的建议
车速不会互相矛盾。坐标轴、刻度、图例都是手工绘制的，因此只需要 numpy + Pillow。
"""

from __future__ import annotations

import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

#: 画布尺寸与配色。
CANVAS = (1600, 1200)
BACKGROUND = "#FFFFFF"
PANEL_FILL = "#F8FAFC"
PANEL_EDGE = "#CBD5E1"
GRID = "#E2E8F0"
TEXT = "#0F172A"
MUTED = "#64748B"
ALERT = "#B91C1C"
THRESHOLD = "#94A3B8"
ACCEPTABLE = "#D97706"
SPEED_COLORS = ("#1A73E8", "#E8710A", "#0F9D58")

_CN_FONTS = (
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)


def _font_path():
    for candidate in _CN_FONTS:
        if os.path.exists(candidate):
            return candidate
    return None


_FONT_CACHE: dict = {}


def _font(size: int):
    if size not in _FONT_CACHE:
        path = _font_path()
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


def _fmt(value: float, digits: int = 2) -> str:
    """紧凑格式：去掉多余的 0，用于刻度与轴名。"""
    text = f"{float(value):.{digits}f}"
    return text.rstrip("0").rstrip(".") if "." in text else text


def _num(value: float, digits: int = 2) -> str:
    """固定小数位，保证表格里的数字对齐。"""
    return f"{float(value):.{digits}f}"


def _ticks(low: float, high: float, count: int = 6):
    return [low + (high - low) * index / (count - 1) for index in range(count)]


def _nice_step(span: float) -> float:
    """把区间跨度换成 1/2/5×10^n 的刻度步长。"""
    raw = span / 5.0
    magnitude = 10.0 ** math.floor(math.log10(raw))
    for factor in (1.0, 2.0, 5.0, 10.0):
        if raw <= factor * magnitude:
            return factor * magnitude
    return 10.0 * magnitude


class _Panel:
    """一块数据绘图区：负责坐标变换、网格、刻度与折线。"""

    def __init__(self, draw, box, xlim, ylim, title, xlabel, ylabel,
                 xdigits=1, ydigits=1, yticks=None) -> None:
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
        draw.text(((self.left + self.right - width) / 2.0, self.bottom + 30),
                  xlabel, fill=MUTED, font=_font(15))
        if yticks is None:
            step = _nice_step(self.ylim[1] - self.ylim[0])
            start = math.ceil(self.ylim[0] / step) * step
            yticks = []
            value = start
            while value <= self.ylim[1] + 1e-9:
                yticks.append(round(value, 10))
                value += step
        self._grid(_ticks(self.xlim[0], self.xlim[1], 6), yticks)

    # ---------------------------------------------------------------- 坐标变换
    def px(self, x: float) -> float:
        low, high = self.xlim
        return self.left + (float(x) - low) / (high - low) * (self.right - self.left)

    def py(self, y: float) -> float:
        low, high = self.ylim
        return self.bottom - (float(y) - low) / (high - low) * (self.bottom - self.top)

    # ------------------------------------------------------------------- 装饰
    def _grid(self, xticks, yticks) -> None:
        for value in xticks:
            if not self.xlim[0] <= value <= self.xlim[1]:
                continue
            x = self.px(value)
            self.draw.line([(x, self.top), (x, self.bottom)], fill=GRID, width=1)
            label = _fmt(value, self.xdigits)
            width = self.draw.textlength(label, font=_font(14))
            self.draw.text((x - width / 2.0, self.bottom + 7), label,
                           fill=MUTED, font=_font(14))
        for value in yticks:
            if not self.ylim[0] <= value <= self.ylim[1]:
                continue
            y = self.py(value)
            self.draw.line([(self.left, y), (self.right, y)], fill=GRID, width=1)
            label = _fmt(value, self.ydigits)
            width = self.draw.textlength(label, font=_font(14))
            self.draw.text((self.left - 9 - width, y - 8), label,
                           fill=MUTED, font=_font(14))

    def line(self, xs, ys, color, width=3) -> None:
        points = []
        for x, y in zip(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)):
            if not (self.xlim[0] <= x <= self.xlim[1] and self.ylim[0] <= y <= self.ylim[1]):
                continue
            points.append((self.px(x), self.py(y)))
        if len(points) > 1:
            self.draw.line(points, fill=color, width=width, joint="curve")

    def hline(self, y, color, width=2, dashed=True) -> None:
        start = (self.left, self.py(y))
        end = (self.right, self.py(y))
        if dashed:
            _dashed(self.draw, start, end, color, width=width)
        else:
            self.draw.line([start, end], fill=color, width=width)

    def vline(self, x, color, width=2, dashed=True) -> None:
        start = (self.px(x), self.top)
        end = (self.px(x), self.bottom)
        if dashed:
            _dashed(self.draw, start, end, color, width=width)
        else:
            self.draw.line([start, end], fill=color, width=width)

    def badge(self, x, y, text, color) -> None:
        """在数据坐标处画一个带底色的标注（左上角对齐）。"""
        font = _font(15)
        width = self.draw.textlength(text, font=font)
        px = min(self.px(x), self.right - width - 12)
        py = self.py(y)
        self.draw.rectangle([(px - 5, py - 4), (px + width + 7, py + 22)],
                            fill=BACKGROUND, outline=color)
        self.draw.text((px + 1, py), text, fill=color, font=font)


def _legend(draw, origin, entries) -> None:
    x, y = origin
    for color, label in entries:
        draw.line([(x, y + 10), (x + 28, y + 10)], fill=color, width=4)
        draw.text((x + 36, y), label, fill=TEXT, font=_font(16))
        x += 44 + draw.textlength(label, font=_font(16))


def render(demo: dict, path: str) -> str:
    """把 ``build_demo`` 的结果画成 PNG 并写到 ``path``。"""
    image = Image.new("RGB", CANVAS, BACKGROUND)
    draw = ImageDraw.Draw(image)

    bump = demo["bump"]
    vehicle = demo["vehicle"]
    rows = demo["rows"]
    curve = demo["curve"]
    limits = demo["limits_mps2"]
    advice = demo["advice"]
    passengers = demo["passages"]

    draw.text((40, 22), "减速带通过平顺性与建议车速评估", fill=TEXT, font=_font(32))
    subtitle = (
        f"{_fmt(bump.height_m * 1000, 0)} mm 高 × {_fmt(bump.width_m, 1)} m 宽"
        f"（{'半正弦' if bump.kind == 'half_sine' else '梯形'}），四分之一车模型："
        f"车身 {_fmt(vehicle.sprung_mass_kg, 0)} kg + 非悬挂 {_fmt(vehicle.unsprung_mass_kg, 0)} kg，"
        f"车身偏频 {_fmt(vehicle.body_natural_frequency_hz, 2)} Hz，阻尼比 {_fmt(vehicle.damping_ratio, 2)}"
    )
    draw.text((40, 70), subtitle, fill=MUTED, font=_font(17))
    draw.line([(40, 104), (CANVAS[0] - 40, 104)], fill=PANEL_EDGE, width=2)

    window = 1.0  # 子图 a/b 显示的时间窗口（s）

    # ------------------------------------------------ (a) 车身垂向加速度
    accel_all = np.concatenate(
        [p.sprung_accel_mps2[p.time_s <= window] for p in passengers]
    )
    accel_limit = math.ceil((float(np.max(np.abs(accel_all))) + 1.2) / 2.0) * 2.0
    panel_a = _Panel(
        draw,
        (110, 170, 770, 540),
        xlim=(0.0, window),
        ylim=(-accel_limit, accel_limit),
        title="(a) 车身垂向加速度时程",
        xlabel="时间 t / s（t = 0 为车轮到达减速带前沿）",
        ylabel="a_s / (m/s²)   虚线为舒适/可接受阈值",
        xdigits=1,
        ydigits=0,
        yticks=_ticks(-accel_limit, accel_limit, 5),
    )
    for _, limit in limits:
        panel_a.hline(limit, THRESHOLD)
        panel_a.hline(-limit, THRESHOLD)
    for color, passage in zip(SPEED_COLORS, passengers):
        mask = passage.time_s <= window
        panel_a.line(passage.time_s[mask], passage.sprung_accel_mps2[mask], color)
    _legend(
        draw,
        (110, 598),
        [
            (color, f"{_num(p.speed_mps * 3.6, 1)} km/h")
            for color, p in zip(SPEED_COLORS, passengers)
        ],
    )

    # ------------------------------------------------ (b) 轮胎动载比
    ratio_all = np.concatenate(
        [p.tire_load_ratio[p.time_s <= window] for p in passengers]
    )
    ratio_low = min(-0.3, math.floor((float(np.min(ratio_all)) - 0.1) * 10) / 10)
    ratio_high = max(1.6, math.ceil((float(np.max(ratio_all)) + 0.1) * 10) / 10)
    panel_b = _Panel(
        draw,
        (890, 170, 1550, 540),
        xlim=(0.0, window),
        ylim=(ratio_low, ratio_high),
        title="(b) 轮胎动载比 F_z / F_z,static",
        xlabel="时间 t / s（放大看轮荷波动）",
        ylabel="动载比：1.0 为静载，≤ 0 表示车轮离地（红色虚线）",
        xdigits=1,
        ydigits=1,
    )
    panel_b.hline(1.0, THRESHOLD, width=2)
    panel_b.hline(0.0, ALERT, width=2)
    for color, passage in zip(SPEED_COLORS, passengers):
        mask = passage.time_s <= window
        panel_b.line(passage.time_s[mask], passage.tire_load_ratio[mask], color)
    lift = [
        f"{_num(p.speed_mps * 3.6, 1)} km/h 最小动载比 "
        f"{_num(float(np.min(p.tire_load_ratio)), 2)}"
        for p in passengers
        if float(np.min(p.tire_load_ratio)) <= 0.0
    ]
    draw.text(
        (890, 598),
        "车轮离地：" + "；".join(lift) if lift else "三档车速均未离地",
        fill=ALERT if lift else MUTED,
        font=_font(15),
    )

    # ------------------------------------------------ (c) 峰值加速度随车速
    curve_speeds = np.asarray(curve["speeds_mps"], dtype=float)
    curve_peaks = np.asarray(curve["peak_accel_mps2"], dtype=float)
    shown = curve_speeds <= 12.0
    peak_limit = math.ceil((float(np.max(curve_peaks[shown])) + 0.8) / 2.0) * 2.0
    panel_c = _Panel(
        draw,
        (110, 700, 770, 1050),
        xlim=(0.0, 12.0),
        ylim=(0.0, peak_limit),
        title="(c) 峰值加速度随车速变化（扫速 1–12 m/s 展示段）",
        xlabel="车速 / (m/s)",
        ylabel="峰值 |a_s| / (m/s²)",
        xdigits=0,
        ydigits=0,
    )
    for index, (limit_label, limit) in enumerate(limits):
        color = THRESHOLD if index == 0 else ACCEPTABLE
        panel_c.hline(limit, color)
        label = f"{limit_label}线 {_fmt(limit, 1)} m/s²"
        draw.text((panel_c.right - 20 - draw.textlength(label, font=_font(15)),
                   panel_c.py(limit) + 6), label, fill=color, font=_font(15))
    panel_c.line(curve_speeds[shown], curve_peaks[shown], SPEED_COLORS[0], width=3)
    for index, entry in enumerate(advice):
        if entry.get("speed_mps") is None:
            continue
        color = THRESHOLD if index == 0 else ACCEPTABLE
        panel_c.vline(entry["speed_mps"], color)
        panel_c.badge(
            entry["speed_mps"] + 0.12,
            entry["limit_mps2"] + 0.9,
            f"{entry['label']} ≤ {_num(entry['speed_mps'] * 3.6, 1)} km/h",
            color,
        )
    draw.text(
        (110, 1116),
        f"完整扫速到 {_fmt(float(curve_speeds[-1]), 0)} m/s；更高车速段峰值已超出本图纵轴",
        fill=MUTED,
        font=_font(15),
    )

    # ------------------------------------------------ (d) 指标表
    table = (890, 700, 1550, 1050)
    draw.rectangle(table, fill=PANEL_FILL, outline=PANEL_EDGE, width=2)
    draw.text((table[0], table[1] - 50), "(d) 三档车速的通过指标", fill=TEXT, font=_font(20))
    columns = (
        ("车速 km/h", 16, 96, False),
        ("峰值 m/s²", 118, 96, True),
        ("RMS m/s²", 220, 92, True),
        ("VDV m/s^1.75", 318, 122, True),
        ("最小轮荷比", 446, 96, True),
        ("动行程 mm", 548, 104, True),
    )
    header_y = table[1] + 16
    for name, offset, width, numeric in columns:
        text_width = draw.textlength(name, font=_font(15))
        text_x = table[0] + offset + width - text_width if numeric else table[0] + offset
        draw.text((text_x, header_y), name, fill=MUTED, font=_font(15))
    draw.line([(table[0] + 12, header_y + 28), (table[2] - 12, header_y + 28)],
              fill=PANEL_EDGE, width=2)
    for index, row in enumerate(rows):
        y = header_y + 44 + index * 40
        values = (
            (_num(row["speed_kmh"], 1), False),
            (_num(row["peak_accel_mps2"], 2), row["exceeds_comfort_limit"]),
            (_num(row["rms_accel_mps2"], 2), False),
            (_num(row["vdv_mps1_75"], 2), False),
            (_num(row["tire_load_min_ratio"], 2), row["tire_load_min_ratio"] <= 0.0),
            (_num(row["suspension_travel_mm"], 1),
             bool(row.get("exceeds_travel_limit"))),
        )
        draw.rectangle([(table[0] + 4, y + 1), (table[0] + 9, y + 21)],
                       fill=SPEED_COLORS[index % len(SPEED_COLORS)])
        for (name, offset, width, _), (text, flagged) in zip(columns, values):
            color = ALERT if flagged else TEXT
            right = table[0] + offset + width
            text_width = draw.textlength(text, font=_font(16))
            draw.text((right - text_width, y), text, fill=color,
                      font=_font(17) if name == "车速 km/h" else _font(16))

    lines = ["建议通过车速（自定义阈值，非标准限值，非 ISO 2631 限值）："]
    for entry in advice:
        if entry.get("speed_mps") is None:
            lines.append(
                f"  · {entry['label']}：扫速范围内无满足项"
                f"（{_num(entry['first_speed_mps'], 1)} m/s 已超阈值）"
            )
            continue
        lines.append(
            f"  · {entry['label']}：≤ {_num(entry['speed_mps'] * 3.6, 1)} km/h"
            f"（{_num(entry['speed_mps'], 1)} m/s，峰值 "
            f"{_num(entry['peak_accel_mps2'], 2)} m/s²，阈值 "
            f"{_num(entry['limit_mps2'], 1)} m/s²）"
        )
    lines.append(
        f"最大动行程 {_num(max(r['suspension_travel_mm'] for r in rows), 1)} mm，"
        f"限位 {_num(demo['vehicle_summary']['travel_limit_mm'], 0)} mm；"
        "红色表示超舒适阈值、超行程限位或车轮离地"
    )
    lift_speeds = curve_speeds[np.asarray(curve["tire_load_min_ratio"], dtype=float) <= 0.0]
    lift_note = (
        f"{_num(float(lift_speeds[0]), 1)} m/s 起出现车轮离地"
        if lift_speeds.size
        else "全扫速范围内车轮未离地"
    )
    lines.append(
        f"完整扫速 1–{_num(float(curve_speeds[-1]), 0)} m/s：峰值加速度 "
        f"{_num(float(curve_peaks[0]), 2)} → {_num(float(curve_peaks[-1]), 2)} m/s²；{lift_note}"
    )
    for index, text in enumerate(lines):
        draw.text((table[0] + 14, table[1] + 186 + index * 30), text,
                  fill=MUTED if index == 0 else TEXT,
                  font=_font(15) if index == 0 else _font(16))

    footer = (
        "数据来源：python main.py demo（线性四分之一车模型，1 ms 定步长 RK4；"
        "未含俯仰、悬架限位与减振器非线性；指标定义见 docs/ground/speed_bump_ride_comfort.md）"
    )
    draw.text((40, CANVAS[1] - 42), footer, fill=MUTED, font=_font(14))

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    image.save(path, format="PNG", optimize=True)
    return path
