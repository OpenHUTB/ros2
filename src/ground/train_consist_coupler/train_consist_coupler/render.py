"""极简 RGB 画布与 PNG 编码，只用标准库（zlib + struct）。

这样做的原因和 train_ato_stop 相同：只要装了 Python 就能出图，评审换一台干净机器
也能复现，不用装 matplotlib。
"""

import struct
import zlib

from . import font5x7


class Canvas:
    def __init__(self, width, height, background=(255, 255, 255)):
        self.w = int(width)
        self.h = int(height)
        self.buf = bytearray(bytes(background) * (self.w * self.h))

    # ---------- 基本绘图 ----------
    def px(self, x, y, color):
        x = int(x)
        y = int(y)
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.buf[i] = color[0]
            self.buf[i + 1] = color[1]
            self.buf[i + 2] = color[2]

    def fill(self, x, y, w, h, color):
        x0, y0 = max(0, int(x)), max(0, int(y))
        x1, y1 = min(self.w, int(x) + int(w)), min(self.h, int(y) + int(h))
        for yy in range(y0, y1):
            base = (yy * self.w + x0) * 3
            row = bytes(color) * (x1 - x0)
            self.buf[base:base + len(row)] = row

    def hline(self, x0, x1, y, color):
        if x0 > x1:
            x0, x1 = x1, x0
        self.fill(x0, y, x1 - x0 + 1, 1, color)

    def vline(self, x, y0, y1, color):
        if y0 > y1:
            y0, y1 = y1, y0
        self.fill(x, y0, 1, y1 - y0 + 1, color)

    def rect(self, x, y, w, h, color):
        self.hline(x, x + w - 1, y, color)
        self.hline(x, x + w - 1, y + h - 1, color)
        self.vline(x, y, y + h - 1, color)
        self.vline(x + w - 1, y, y + h - 1, color)

    def line(self, x0, y0, x1, y1, color, width=1):
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        half = (width - 1) // 2
        while True:
            if width <= 1:
                self.px(x0, y0, color)
            else:
                self.fill(x0 - half, y0 - half, width, width, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def dashed(self, x0, y0, x1, y1, color, dash=8, gap=6, width=1):
        dx = x1 - x0
        dy = y1 - y0
        length = max(abs(dx), abs(dy))
        if length < 1:
            return
        period = dash + gap
        step = 0
        while step <= length:
            if step % period < dash:
                t0 = step / length
                t1 = min(length, step + 1) / length
                self.line(x0 + dx * t0, y0 + dy * t0,
                          x0 + dx * t1, y0 + dy * t1, color, width)
            step += 1

    # ---------- 文字 ----------
    def text_width(self, s, scale=2):
        return max(0, (len(s) * font5x7.ADVANCE - 1) * scale)

    def text(self, s, x, y, color, scale=2):
        cx = int(x)
        for ch in s:
            glyph = font5x7.rows(ch)
            for row_index, bits in enumerate(glyph[:7]):
                for col_index, bit in enumerate(bits[:5]):
                    if bit == "1":
                        self.fill(cx + col_index * scale, int(y) + row_index * scale,
                                  scale, scale, color)
            cx += font5x7.ADVANCE * scale

    def text_right(self, s, x_right, y, color, scale=2):
        self.text(s, x_right - self.text_width(s, scale), y, color, scale)

    def text_center(self, s, x_center, y, color, scale=2):
        self.text(s, x_center - self.text_width(s, scale) // 2, y, color, scale)

    # ---------- 输出 ----------
    def save_png(self, path):
        stride = self.w * 3
        raw = bytearray()
        for y in range(self.h):
            raw.append(0)  # filter type 0
            raw += self.buf[y * stride:(y + 1) * stride]
        compressed = zlib.compress(bytes(raw), 9)

        def chunk(tag, data):
            body = tag + data
            return (struct.pack(">I", len(data)) + body
                    + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

        header = struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0)
        png = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", header)
               + chunk(b"IDAT", compressed)
               + chunk(b"IEND", b""))
        with open(path, "wb") as fp:
            fp.write(png)


# 配色（浅色主题，白底）
INK = (32, 38, 46)
MUTED = (128, 138, 148)
GRID = (226, 231, 236)
AXIS = (150, 158, 168)
BLUE = (32, 96, 176)
RED = (196, 48, 43)
GREEN = (36, 138, 84)
ORANGE = (222, 125, 24)
PURPLE = (112, 78, 160)
TEAL = (20, 138, 140)
