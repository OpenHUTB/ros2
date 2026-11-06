# -*- coding: utf-8 -*-
"""极简 PNG 折线图输出（只用标准库 zlib）。

为什么手写而不用 matplotlib
---------------------------
本模块要能在没有 GUI、没有额外依赖的环境里出图（例如只装了 Python 的
服务器或虚拟机）。PNG 的最小结构只有 IHDR / IDAT / IEND 三个块，
用 zlib 压缩像素行即可生成，因此这里自己实现一个够用的版本。

支持：白底、坐标轴、折线、图例文字（5x7 点阵）、标题。
"""

from __future__ import annotations

import struct
import zlib

__all__ = ["Canvas", "plot_series"]

# 5x7 点阵字体：只收录绘图标题与图例用得到的字符
_FONT = {
    " ": ["00000"] * 7,
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    "+": ["00000", "00100", "00100", "11111", "00100", "00100", "00000"],
    ":": ["00000", "01100", "01100", "00000", "01100", "01100", "00000"],
    "(": ["00010", "00100", "01000", "01000", "01000", "00100", "00010"],
    ")": ["01000", "00100", "00010", "00010", "00010", "00100", "01000"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "=": ["00000", "00000", "11111", "00000", "11111", "00000", "00000"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01110", "10001", "10000", "10111", "10001", "10001", "01111"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["01110", "00100", "00100", "00100", "00100", "00100", "01110"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "11011", "10001"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
}

_BLACK = (0, 0, 0)
_WHITE = (255, 255, 255)
_GRID = (225, 225, 225)
_AXIS = (120, 120, 120)


class Canvas(object):
    """简单的 RGB 像素画布。"""

    def __init__(self, width, height, background=_WHITE):
        if width <= 0 or height <= 0:
            raise ValueError("画布尺寸必须为正")
        self.w = int(width)
        self.h = int(height)
        self.bg = background
        self.pixels = [[background] * self.w for _ in range(self.h)]

    def set(self, x, y, color):
        x, y = int(x), int(y)
        if 0 <= x < self.w and 0 <= y < self.h:
            self.pixels[y][x] = color

    def line(self, x0, y0, x1, y1, color, thickness=1):
        """Bresenham 直线。thickness>1 时向四周加粗。"""
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        half = thickness // 2
        while True:
            for ox in range(-half, half + 1):
                for oy in range(-half, half + 1):
                    self.set(x0 + ox, y0 + oy, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def rect(self, x0, y0, x1, y1, color):
        self.line(x0, y0, x1, y0, color)
        self.line(x1, y0, x1, y1, color)
        self.line(x1, y1, x0, y1, color)
        self.line(x0, y1, x0, y0, color)

    def text(self, x, y, s, color=_BLACK, scale=1):
        """用 5x7 点阵写字符串，未知字符按空格处理。"""
        cx = int(x)
        for ch in s:
            glyph = _FONT.get(ch.upper())
            if glyph is None:
                cx += 6 * scale
                continue
            for row, bits in enumerate(glyph):
                for col, bit in enumerate(bits):
                    if bit == "1":
                        for oy in range(scale):
                            for ox in range(scale):
                                self.set(cx + col * scale + ox,
                                         y + row * scale + oy, color)
            cx += 6 * scale

    def to_png_bytes(self):
        raw = bytearray()
        for row in self.pixels:
            raw.append(0)  # 每行前缀 filter type = 0 (None)
            for (r, g, b) in row:
                raw.append(r)
                raw.append(g)
                raw.append(b)

        def chunk(tag, data):
            out = struct.pack(">I", len(data)) + tag + data
            out += struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
            return out

        ihdr = struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0)
        return (b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", ihdr)
                + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
                + chunk(b"IEND", b""))

    def save(self, path):
        with open(path, "wb") as fh:
            fh.write(self.to_png_bytes())
        return path


# ---------------------------------------------------------------------------
def plot_series(path, series, title="", width=900, height=560,
                margin=(60, 30, 30, 150), ylabel=""):
    """画一组折线并存成 PNG。

    参数
    ----
    path : 输出文件路径
    series : [(标签, xs, ys, (r,g,b)), ...]
    title : 标题（5x7 点阵，只支持 ASCII）
    margin : (左, 上, 右, 下) 像素边距
    ylabel : 纵轴说明（ASCII）
    """
    if not series:
        raise ValueError("series 不能为空")

    left, top, right, bottom = margin
    plot_w = width - left - right
    plot_h = height - top - bottom
    if plot_w <= 0 or plot_h <= 0:
        raise ValueError("边距过大，绘图区为空")

    # 自动量程
    all_x = [x for (_l, xs, _ys, _c) in series for x in xs]
    all_y = [y for (_l, _xs, ys, _c) in series for y in ys]
    if not all_x or not all_y:
        raise ValueError("折线数据为空")
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    if x_max - x_min < 1e-12:
        x_max = x_min + 1.0
    if y_max - y_min < 1e-12:
        y_max = y_min + 1.0
    pad = (y_max - y_min) * 0.08
    y_min -= pad
    y_max += pad

    def sx(x):
        return left + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y):
        return top + plot_h - (y - y_min) / (y_max - y_min) * plot_h

    c = Canvas(width, height)

    # 网格与坐标轴
    for i in range(6):
        gy = top + plot_h * i / 5.0
        c.line(left, gy, left + plot_w, gy, _GRID)
    for i in range(6):
        gx = left + plot_w * i / 5.0
        c.line(gx, top, gx, top + plot_h, _GRID)
    c.rect(left, top, left + plot_w, top + plot_h, _AXIS)

    # 纵轴刻度
    for i in range(6):
        val = y_min + (y_max - y_min) * (5 - i) / 5.0
        c.text(4, top + plot_h * i / 5.0 - 3, "%.2f" % val, _AXIS)

    # 折线
    for (_label, xs, ys, color) in series:
        for i in range(1, len(xs)):
            c.line(sx(xs[i - 1]), sy(ys[i - 1]), sx(xs[i]), sy(ys[i]), color, 2)

    # 标题
    if title:
        c.text(left, 8, title, _BLACK, 2)

    # 图例
    ly = top + plot_h + 16
    for (label, _xs, _ys, color) in series:
        c.line(left, ly + 3, left + 26, ly + 3, color, 3)
        c.text(left + 32, ly, label, _BLACK)
        ly += 16

    if ylabel:
        c.text(left, top + plot_h + 16 + 16 * len(series) + 8, ylabel, _AXIS)

    return c.save(path)
