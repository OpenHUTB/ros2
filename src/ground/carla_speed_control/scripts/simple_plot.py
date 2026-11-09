#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""极简 PNG 绘图器（纯 numpy + zlib，不依赖 matplotlib / Pillow）。

为什么自己写
------------
本模块要求"只用 numpy 就能运行"，以便在教学虚拟机上直接验证。但作业与 PR
需要**图片形式**的结果（截图、文档配图），而 matplotlib / Pillow 不在依赖里。
与其引入依赖，不如自己生成 PNG：

* PNG 文件结构很简单（IHDR + IDAT + IEND），IDAT 只需对扫描线做 zlib 压缩；
* 只支持 8 位 RGB，够画曲线图；
* 绘图本身用遍历像素的直线算法，几百条线段的性能完全够用。

这样任何有 numpy 的机器都能出图，且不增加部署负担。

用法::

    img = Canvas(800, 600, bg=(255, 255, 255))
    img.line(10, 20, 300, 400, color=(255, 0, 0), width=2)
    img.save("out.png")
"""

from __future__ import annotations

import struct
import zlib
from typing import List, Sequence, Tuple

import numpy as np

__all__ = ["Canvas", "PALETTE"]

#: 默认配色（与常见绘图库风格接近），按需取用
PALETTE = {
    "blue": (31, 119, 180),
    "orange": (255, 127, 14),
    "green": (44, 160, 44),
    "red": (214, 39, 40),
    "purple": (148, 103, 189),
    "gray": (127, 127, 127),
    "lightgray": (200, 200, 200),
    "black": (0, 0, 0),
    "white": (255, 255, 255),
}

#: 5x7 点阵字库，仅含绘图需要的字符（数字、字母、常用符号）
_FONT = {
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
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "(": ["00010", "00100", "01000", "01000", "01000", "00100", "00010"],
    ")": ["01000", "00100", "00010", "00010", "00010", "00100", "01000"],
    "=": ["00000", "00000", "11111", "00000", "11111", "00000", "00000"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
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
    "_": ["00000", "00000", "00000", "00000", "00000", "00000", "11111"],
}


class Canvas:
    """一张 RGB 位图，提供最基本的绘图原语。"""

    def __init__(self, width: int, height: int, bg=(255, 255, 255)):
        if width <= 0 or height <= 0:
            raise ValueError("画布尺寸必须为正")
        self.w = int(width)
        self.h = int(height)
        self.buf = np.empty((self.h, self.w, 3), dtype=np.uint8)
        self.buf[:, :] = np.asarray(bg, dtype=np.uint8)

    # -- 基本绘制 ---------------------------------------------------------

    def set_pixel(self, x: int, y: int, color) -> None:
        if 0 <= x < self.w and 0 <= y < self.h:
            self.buf[y, x] = np.asarray(color, dtype=np.uint8)

    def rect(self, x0: int, y0: int, x1: int, y1: int, color, fill: bool = False,
             width: int = 1) -> None:
        """画矩形。fill=True 填充，否则只画边框。"""
        x0, x1 = sorted((int(x0), int(x1)))
        y0, y1 = sorted((int(y0), int(y1)))
        x0 = max(0, x0); y0 = max(0, y0)
        x1 = min(self.w - 1, x1); y1 = min(self.h - 1, y1)
        if x0 > x1 or y0 > y1:
            return
        c = np.asarray(color, dtype=np.uint8)
        if fill:
            self.buf[y0:y1 + 1, x0:x1 + 1] = c
            return
        wdt = max(1, int(width))
        for k in range(wdt):
            if y0 + k <= y1:
                self.buf[y0 + k, x0:x1 + 1] = c
            if y1 - k >= y0:
                self.buf[y1 - k, x0:x1 + 1] = c
            if x0 + k <= x1:
                self.buf[y0:y1 + 1, x0 + k] = c
            if x1 - k >= x0:
                self.buf[y0:y1 + 1, x1 - k] = c

    def line(self, x0: float, y0: float, x1: float, y1: float, color,
             width: int = 1) -> None:
        """画线段（Bresenham 的浮点变体，支持线宽）。"""
        x0, y0, x1, y1 = float(x0), float(y0), float(x1), float(y1)
        if not all(np.isfinite([x0, y0, x1, y1])):
            return
        c = np.asarray(color, dtype=np.uint8)
        wdt = max(1, int(width))
        half = wdt // 2
        steps = int(max(abs(x1 - x0), abs(y1 - y0)))
        if steps <= 0:
            for dy in range(-half, half + 1):
                for dx in range(-half, half + 1):
                    self.set_pixel(int(round(x0)) + dx, int(round(y0)) + dy, c)
            return
        for i in range(steps + 1):
            t = i / steps
            x = int(round(x0 + (x1 - x0) * t))
            y = int(round(y0 + (y1 - y0) * t))
            ya, yb = max(0, y - half), min(self.h, y + half + 1)
            xa, xb = max(0, x - half), min(self.w, x + half + 1)
            if ya < yb and xa < xb:
                self.buf[ya:yb, xa:xb] = c

    def polyline(self, xs: Sequence[float], ys: Sequence[float], color,
                 width: int = 1) -> None:
        """画折线。"""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        n = min(xs.size, ys.size)
        for i in range(n - 1):
            self.line(xs[i], ys[i], xs[i + 1], ys[i + 1], color, width)

    def text(self, x: int, y: int, s: str, color, scale: int = 1) -> None:
        """用内置点阵字库画 ASCII 文本。不支持中文（避免引入字体依赖）。"""
        c = np.asarray(color, dtype=np.uint8)
        cx = int(x)
        for ch in str(s):
            glyph = _FONT.get(ch.upper())
            if glyph is None:
                cx += 6 * scale
                continue
            for row, bits in enumerate(glyph):
                for col, bit in enumerate(bits):
                    if bit == "1":
                        px = cx + col * scale
                        py = int(y) + row * scale
                        for dy in range(scale):
                            for dx in range(scale):
                                self.set_pixel(px + dx, py + dy, c)
            cx += 6 * scale

    def text_centered(self, cx: int, y: int, s: str, color, scale: int = 1) -> None:
        """按中心 x 居中画文本。"""
        width = len(str(s)) * 6 * scale
        self.text(int(cx - width // 2), y, s, color, scale)

    # -- 保存 -------------------------------------------------------------

    def to_png_bytes(self) -> bytes:
        """按 PNG 规范编码（8 位 RGB，无滤波）。"""
        raw = bytearray()
        # 每行前置一个滤波类型字节 0
        for row in range(self.h):
            raw.append(0)
            raw.extend(self.buf[row].tobytes())

        def chunk(tag: bytes, data: bytes) -> bytes:
            out = struct.pack(">I", len(data)) + tag + data
            crc = zlib.crc32(tag + data) & 0xFFFFFFFF
            return out + struct.pack(">I", crc)

        ihdr = struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0)
        return (
            b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + chunk(b"IEND", b"")
        )

    def save(self, path: str) -> str:
        with open(path, "wb") as fh:
            fh.write(self.to_png_bytes())
        return path


# ---------------------------------------------------------------------------
# 坐标系与绘图辅助
# ---------------------------------------------------------------------------


class Axes:
    """带坐标映射的绘图区，负责把数据坐标映射到画布像素。"""

    def __init__(self, canvas: Canvas, x0: int, y0: int, x1: int, y1: int,
                 xlim, ylim, xlabel="", ylabel="", title=""):
        self.c = canvas
        self.px0, self.py0 = int(x0), int(y0)      # 左上（数据区左上角）
        self.px1, self.py1 = int(x1), int(y1)      # 右下
        self.xlim = (float(xlim[0]), float(xlim[1]))
        self.ylim = (float(ylim[0]), float(ylim[1]))
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.title = title
        if self.xlim[1] <= self.xlim[0]:
            self.xlim = (self.xlim[0], self.xlim[0] + 1.0)
        if self.ylim[1] <= self.ylim[0]:
            self.ylim = (self.ylim[0], self.ylim[0] + 1.0)

    # 数据坐标 -> 像素
    def mx(self, x: float) -> float:
        t = (float(x) - self.xlim[0]) / (self.xlim[1] - self.xlim[0])
        return self.px0 + t * (self.px1 - self.px0)

    def my(self, y: float) -> float:
        t = (float(y) - self.ylim[0]) / (self.ylim[1] - self.ylim[0])
        return self.py1 - t * (self.py1 - self.py0)

    def setup(self, grid_color=(220, 220, 220), axis_color=(0, 0, 0),
              n_x_ticks=6, n_y_ticks=6, tick_fmt="{:.1f}", label_scale=1):
        """画边框、网格、刻度与轴标签。

        布局上做了几处必要的间隔，避免文字互相覆盖：
        * y 轴标签放在绘图区**上方**，与顶端刻度留出距离；
        * y 刻度文字右对齐到轴的左侧，不压住边框；
        * x 刻度文字在轴下方，x 轴标签再往下 22 像素。
        因此调用方需要为左边距（≥ 与标签等宽）和上下边距留足空间，
        见 demo 中的画布尺寸设置。
        """
        c = self.c
        gs = max(1, label_scale)
        # 边框
        c.rect(self.px0, self.py0, self.px1, self.py1, axis_color, width=1)

        # 纵向网格与 x 刻度（文字在轴下方，留 5 像素）
        for i in range(n_x_ticks + 1):
            xv = self.xlim[0] + (self.xlim[1] - self.xlim[0]) * i / n_x_ticks
            xp = int(round(self.mx(xv)))
            c.line(xp, self.py0, xp, self.py1, grid_color, 1)
            c.line(xp, self.py1, xp, self.py1 + 4, axis_color, 1)
            c.text_centered(xp, self.py1 + 9, tick_fmt.format(xv), axis_color, gs)

        # 横向网格与 y 刻度（文字右对齐到轴左侧，留 6 像素）
        for i in range(n_y_ticks + 1):
            yv = self.ylim[0] + (self.ylim[1] - self.ylim[0]) * i / n_y_ticks
            yp = int(round(self.my(yv)))
            c.line(self.px0, yp, self.px1, yp, grid_color, 1)
            c.line(self.px0 - 4, yp, self.px0, yp, axis_color, 1)
            label = tick_fmt.format(yv)
            label_w = len(label) * 6 * gs
            ty = yp - 3 * gs
            # 顶端刻度会与 y 轴标签同高，故不做特殊处理，由调用方留出上方空间
            c.text(self.px0 - 8 - label_w, ty, label, axis_color, gs)

        # 轴标签
        if self.xlabel:
            c.text_centered((self.px0 + self.px1) // 2, self.py1 + 26,
                            self.xlabel, axis_color, gs)
        if self.ylabel:
            # 放在绘图区上方、标题左侧，避免与刻度重叠
            c.text(self.px0 - 8 - len(self.ylabel) * 6 * gs, self.py0 - 14,
                   self.ylabel, axis_color, gs)
        if self.title:
            c.text_centered((self.px0 + self.px1) // 2, self.py0 - 14,
                            self.title, axis_color, gs + 1)

    def plot(self, xs, ys, color, width=2, label=""):
        """画数据折线，并返回用于图例的 (color, label)。"""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        n = min(xs.size, ys.size)
        px = [self.mx(v) for v in xs[:n]]
        py = [self.my(v) for v in ys[:n]]
        self.c.polyline(px, py, color, width)
        return (color, label)

    def hline(self, y, color, width=1):
        yp = int(round(self.my(y)))
        self.c.line(self.px0, yp, self.px1, yp, color, width)

    def legend(self, entries, x=None, y=None, scale=1, box=True):
        """在右上角（或指定位置）画图例。entries: [(color, label), ...]"""
        c = self.c
        items = [(col, lab) for col, lab in entries if lab]
        if not items:
            return
        line_len = 22 * scale
        pad = 6 * scale
        w = max(len(lab) for _col, lab in items) * 6 * scale + line_len + 4 * pad
        h = len(items) * 11 * scale + 2 * pad
        x = self.px1 - w - 8 if x is None else x
        y = self.py0 + 8 if y is None else y
        if box:
            c.rect(x, y, x + w, y + h, (0, 0, 0), fill=False, width=1)
            c.rect(x + 1, y + 1, x + w - 1, y + h - 1, (255, 255, 255), fill=True)
            c.rect(x, y, x + w, y + h, (0, 0, 0), fill=False, width=1)
        for i, (col, lab) in enumerate(items):
            yy = y + pad + i * 11 * scale + 3 * scale
            c.line(x + pad, yy, x + pad + line_len, yy, col, 2 * scale)
            c.text(x + pad + line_len + pad // 2, yy - 3 * scale, lab,
                   (0, 0, 0), scale)
