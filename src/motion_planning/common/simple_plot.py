"""极简折线图输出（仅依赖 zlib / struct，不依赖 matplotlib / numpy / PIL）。

提供 Canvas（像素缓冲 + 基本绘制）与 Figure（坐标轴 / 网格 / 图例 / 标题）。
用于沙箱环境（无第三方包）下生成教学配图。
"""

import zlib
import struct
import math

# ----------------------------------------------------------------------------
# 5x7 位图字体（仅大写 + 数字 + 常用符号；小写自动转大写）。
# 每个字形用 7 行、每行 5 位的字符串表示，'#' 表示点亮。
# ----------------------------------------------------------------------------
_FONT = {
    'A': ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    'B': ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    'C': ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    'D': ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    'E': ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    'F': ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    'G': ["01111", "10000", "10000", "10111", "10001", "10001", "01111"],
    'H': ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    'I': ["01110", "00100", "00100", "00100", "00100", "00100", "01110"],
    'J': ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    'K': ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    'L': ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    'M': ["10001", "11011", "11111", "10101", "10001", "10001", "10001"],
    'N': ["10001", "11001", "11001", "10101", "10011", "10011", "10001"],
    'O': ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    'P': ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    'Q': ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    'R': ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    'S': ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    'T': ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    'U': ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    'V': ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    'W': ["10001", "10001", "10001", "10101", "11111", "11011", "10001"],
    'X': ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    'Y': ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    'Z': ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    '0': ["01110", "10011", "10101", "10101", "11001", "11001", "01110"],
    '1': ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    '2': ["01110", "10001", "00001", "00110", "01000", "10000", "11111"],
    '3': ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    '4': ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    '5': ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    '6': ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    '7': ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    '8': ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    '9': ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
    ' ': ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    '-': ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    '.': ["00000", "00000", "00000", "00000", "00000", "00110", "00110"],
    ':': ["00000", "00110", "00110", "00000", "00110", "00110", "00000"],
    '/': ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    '(': ["00110", "01000", "01000", "01000", "01000", "01000", "00110"],
    ')': ["01100", "00010", "00010", "00010", "00010", "00010", "01100"],
    '%': ["11001", "11010", "00100", "01011", "11001", "00000", "00000"],
    '+': ["00000", "00100", "00100", "11111", "00100", "00100", "00000"],
    '=': ["00000", "11111", "00000", "11111", "00000", "00000", "00000"],
    ',': ["00000", "00000", "00000", "00000", "00110", "00100", "01000"],
    '*': ["00000", "10101", "01110", "11111", "01110", "10101", "00000"],
    "'": ["00100", "00100", "00100", "00000", "00000", "00000", "00000"],
}


class Canvas:
    """像素缓冲 + 基本图元。颜色为 (r,g,b) 三元组。"""

    def __init__(self, w, h, bg=(255, 255, 255)):
        self.w = w
        self.h = h
        self.bg = bg
        self.buf = bytearray(w * h * 3)
        for i in range(w * h):
            self.buf[i * 3] = bg[0]
            self.buf[i * 3 + 1] = bg[1]
            self.buf[i * 3 + 2] = bg[2]

    def set(self, x, y, color):
        x = int(round(x))
        y = int(round(y))
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.buf[i] = color[0]
            self.buf[i + 1] = color[1]
            self.buf[i + 2] = color[2]

    def line(self, x0, y0, x1, y1, color):
        # Bresenham
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.set(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def rect(self, x0, y0, x1, y1, color, fill=False):
        x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
        if x1 < x0:
            x0, x1 = x1, x0
        if y1 < y0:
            y0, y1 = y1, y0
        if fill:
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    self.set(x, y, color)
        else:
            self.line(x0, y0, x1, y0, color)
            self.line(x0, y1, x1, y1, color)
            self.line(x0, y0, x0, y1, color)
            self.line(x1, y0, x1, y1, color)

    def char(self, x, y, ch, color, scale=1):
        glyph = _FONT.get(ch.upper(), _FONT[' '])
        for row in range(7):
            bits = glyph[row]
            for col in range(5):
                if bits[col] in ('1', '#'):
                    if scale == 1:
                        self.set(x + col, y + row, color)
                    else:
                        self.rect(x + col * scale, y + row * scale,
                                  x + col * scale + scale - 1, y + row * scale + scale - 1,
                                  color, fill=True)

    def text(self, x, y, s, color, scale=1):
        cx = int(round(x))
        cy = int(round(y))
        for ch in str(s):
            if ch == '?':
                ch = ' '
            self.char(cx, cy, ch, color, scale)
            cx += (5 + 1) * scale  # 字宽5 + 间距1

    def text_metrics(self, s, scale=1):
        return (len(str(s)) * 6 - 1) * scale, 7 * scale

    def save(self, path):
        raw = bytearray()
        for y in range(self.h):
            raw.append(0)  # filter type 0 (None)
            raw.extend(self.buf[y * self.w * 3:(y + 1) * self.w * 3])
        comp = zlib.compress(bytes(raw), 9)

        def chunk(typ, data):
            c = struct.pack(">I", len(data)) + typ + data
            crc = zlib.crc32(typ + data) & 0xffffffff
            return c + struct.pack(">I", crc)

        sig = b'\x89PNG\r\n\x1a\n'
        ihdr = struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0)  # 8-bit RGB
        png = sig + chunk(b'IHDR', ihdr) + chunk(b'IDAT', comp) + chunk(b'IEND', b'')
        with open(path, 'wb') as f:
            f.write(png)


class Figure:
    """带坐标轴/网格/图例的折线图。"""

    def __init__(self, width=720, height=420, bg=(255, 255, 255),
                 margin_left=72, margin_right=28, margin_top=52, margin_bottom=60):
        self.width = width
        self.height = height
        self.bg = bg
        self.canvas = Canvas(width, height, bg)
        self.ml = margin_left
        self.mr = margin_right
        self.mt = margin_top
        self.mb = margin_bottom
        self.series = []          # (xs, ys, color, label)
        self.xlim = None
        self.ylim = None
        self.xlabel = ''
        self.ylabel = ''
        self.title = ''
        self.legend_on = True
        self.grid_color = (225, 225, 225)
        self.axis_color = (60, 60, 60)
        self.text_color = (30, 30, 30)

    # ---- 颜色调色板 ----
    RED = (200, 30, 30)
    BLUE = (30, 90, 200)
    GREEN = (20, 150, 60)
    ORANGE = (220, 140, 20)
    PURPLE = (140, 40, 160)
    BLACK = (20, 20, 20)
    GRAY = (130, 130, 130)

    def plot(self, xs, ys, color=BLUE, label=None):
        self.series.append((list(xs), list(ys), color, label))

    def set_xlim(self, lo, hi):
        self.xlim = (lo, hi)

    def set_ylim(self, lo, hi):
        self.ylim = (lo, hi)

    def _compute_lims(self):
        if self.xlim is None:
            xs_all = [x for s in self.series for x in s[0]]
            if xs_all:
                lo, hi = min(xs_all), max(xs_all)
                if hi - lo < 1e-9:
                    hi = lo + 1.0
                pad = (hi - lo) * 0.05
                self.xlim = (lo - pad, hi + pad)
            else:
                self.xlim = (0.0, 1.0)
        if self.ylim is None:
            ys_all = [y for s in self.series for y in s[1] if y is not None]
            if ys_all:
                lo, hi = min(ys_all), max(ys_all)
                if hi - lo < 1e-9:
                    hi = lo + 1.0
                pad = (hi - lo) * 0.05
                self.ylim = (lo - pad, hi + pad)
            else:
                self.ylim = (0.0, 1.0)

    def _fmt(self, v):
        a = abs(v)
        if a == 0:
            return "0"
        if a >= 10000 or a < 0.01:
            return "%.1e" % v
        if a >= 10:
            return "%.0f" % v
        if a >= 1:
            return "%.1f" % v
        return "%.2f" % v

    def _draw_axes(self):
        x0, x1 = self.xlim
        y0, y1 = self.ylim
        px0 = self.ml
        px1 = self.width - self.mr
        py0 = self.height - self.mb   # bottom (y min)
        py1 = self.mt                  # top (y max)

        def tx(v):
            return px0 + (v - x0) / (x1 - x0) * (px1 - px0)

        def ty(v):
            return py0 + (v - y0) / (y1 - y0) * (py1 - py0)

        # 网格 + 刻度（4~5 等分）
        nx = 5
        for i in range(nx + 1):
            gx = x0 + (x1 - x0) * i / nx
            X = tx(gx)
            self.canvas.line(X, py0, X, py1, self.grid_color)
            self.canvas.line(X, py0, X, py0 + 4, self.axis_color)
            lab = self._fmt(gx)
            tw, _ = self.canvas.text_metrics(lab, 2)
            self.canvas.text(X - tw / 2, py0 + 10, lab, self.text_color, 2)
        ny = 5
        for i in range(ny + 1):
            gy = y0 + (y1 - y0) * i / ny
            Y = ty(gy)
            self.canvas.line(px0, Y, px1, Y, self.grid_color)
            self.canvas.line(px0 - 4, Y, px0, Y, self.axis_color)
            lab = self._fmt(gy)
            tw, _ = self.canvas.text_metrics(lab, 2)
            self.canvas.text(px0 - 8 - tw, Y - 7, lab, self.text_color, 2)
        # 轴线
        self.canvas.line(px0, py0, px1, py0, self.axis_color)
        self.canvas.line(px0, py0, px0, py1, self.axis_color)
        # 轴标签
        if self.xlabel:
            tw, _ = self.canvas.text_metrics(self.xlabel, 2)
            self.canvas.text((px0 + px1) / 2 - tw / 2, py0 + 30, self.xlabel, self.text_color, 2)
        if self.ylabel:
            # 竖直文字较麻烦，改为在左上角标注
            self.canvas.text(px0 + 6, py1 - 16, self.ylabel, self.text_color, 2)
        return tx, ty

    def _draw_data(self, tx, ty):
        for xs, ys, color, _ in self.series:
            prev = None
            for x, y in zip(xs, ys):
                if y is None:
                    prev = None
                    continue
                p = (tx(x), ty(y))
                if prev is not None:
                    self.canvas.line(prev[0], prev[1], p[0], p[1], color)
                prev = p

    def _draw_legend(self):
        if not self.legend_on:
            return
        items = [(c, l) for _, _, c, l in self.series if l]
        if not items:
            return
        lx = self.width - self.mr - 150
        ly = self.mt + 6
        self.canvas.rect(lx - 8, ly - 6, self.width - self.mr - 4,
                         ly - 6 + len(items) * 18 + 6, (255, 255, 255), fill=True)
        self.canvas.rect(lx - 8, ly - 6, self.width - self.mr - 4,
                         ly - 6 + len(items) * 18 + 6, (180, 180, 180))
        for i, (c, l) in enumerate(items):
            yy = ly + i * 18
            self.canvas.rect(lx, yy, lx + 16, yy + 10, c, fill=True)
            self.canvas.text(lx + 24, yy, l, self.text_color, 2)

    def _draw_title(self):
        if self.title:
            tw, _ = self.canvas.text_metrics(self.title, 2)
            self.canvas.text((self.width - tw) / 2, 12, self.title, self.text_color, 2)

    def save(self, path):
        self._compute_lims()
        tx, ty = self._draw_axes()
        self._draw_data(tx, ty)
        self._draw_legend()
        self._draw_title()
        self.canvas.save(path)


def demo():
    """自检：生成一张示例 PNG，验证编码器可用。"""
    import os
    fig = Figure(width=480, height=300)
    xs = [i * 0.1 for i in range(60)]
    ys1 = [math.sin(x) for x in xs]
    ys2 = [0.5 * math.cos(x) for x in xs]
    fig.plot(xs, ys1, color=Figure.RED, label="sin")
    fig.plot(xs, ys2, color=Figure.BLUE, label="cos")
    fig.xlabel = "t (s)"
    fig.ylabel = "y"
    fig.title = "DEMO"
    out = os.path.join(os.path.dirname(__file__), "demo.png")
    fig.save(out)
    # 校验 PNG 签名
    with open(out, "rb") as f:
        head = f.read(8)
    ok = head == b'\x89PNG\r\n\x1a\n'
    print("demo png:", out, "valid_png=", ok, "bytes=", os.path.getsize(out))


if __name__ == "__main__":
    demo()
