"""俯视海图渲染，只用标准库：索引色画布 + 5x7 点阵字 + PNG/GIF 编码。

不依赖 PIL、numpy、matplotlib，也不需要 ROS 或图形界面，
所以在任何一台只装了 Python 3 的机器上都能重新生成图片。
"""
from __future__ import annotations

import math
import struct
import zlib

# ---------------------------------------------------------------- 调色板
PALETTE = [
    (244, 247, 250),   # 0 背景
    (221, 229, 236),   # 1 网格
    (28, 82, 145),     # 2 A 船轨迹 / 主色
    (198, 84, 44),     # 3 B 船轨迹
    (68, 133, 60),     # 4 C 船轨迹
    (38, 46, 58),      # 5 船体
    (146, 176, 208),   # 6 安全圈
    (30, 36, 42),      # 7 文字
    (192, 58, 48),     # 8 警示红
    (255, 255, 255),   # 9 白
    (203, 214, 224),   # 10 浅影
    (118, 129, 140),   # 11 辅助线
    (240, 244, 248),   # 12 图例底
    (12, 120, 110),    # 13 让路船标记
    (150, 105, 30),    # 14 直航船标记
    (176, 186, 196),   # 15 边界
]

BG, GRID, C_A, C_B, C_C, HULL, SAFE_RING, TEXT = 0, 1, 2, 3, 4, 5, 6, 7
WARN, WHITE, SHADE, MUTED, LEGEND_BG, GIVE_WAY, STAND_ON, EDGE = 8, 9, 10, 11, 12, 13, 14, 15

SHIP_COLORS = [C_A, C_B, C_C, C_A, C_B]

# ---------------------------------------------------------------- 5x7 点阵字
_FONT = {
    'A': ('01110', '10001', '10001', '11111', '10001', '10001', '10001'),
    'B': ('11110', '10001', '10001', '11110', '10001', '10001', '11110'),
    'C': ('01110', '10001', '10000', '10000', '10000', '10001', '01110'),
    'D': ('11110', '10001', '10001', '10001', '10001', '10001', '11110'),
    'E': ('11111', '10000', '10000', '11110', '10000', '10000', '11111'),
    'F': ('11111', '10000', '10000', '11110', '10000', '10000', '10000'),
    'G': ('01110', '10001', '10000', '10111', '10001', '10001', '01111'),
    'H': ('10001', '10001', '10001', '11111', '10001', '10001', '10001'),
    'I': ('01110', '00100', '00100', '00100', '00100', '00100', '01110'),
    'J': ('00111', '00010', '00010', '00010', '00010', '10010', '01100'),
    'K': ('10001', '10010', '10100', '11000', '10100', '10010', '10001'),
    'L': ('10000', '10000', '10000', '10000', '10000', '10000', '11111'),
    'M': ('10001', '11011', '10101', '10101', '10001', '10001', '10001'),
    'N': ('10001', '11001', '10101', '10011', '10001', '10001', '10001'),
    'O': ('01110', '10001', '10001', '10001', '10001', '10001', '01110'),
    'P': ('11110', '10001', '10001', '11110', '10000', '10000', '10000'),
    'Q': ('01110', '10001', '10001', '10001', '10101', '10010', '01101'),
    'R': ('11110', '10001', '10001', '11110', '10100', '10010', '10001'),
    'S': ('01111', '10000', '10000', '01110', '00001', '00001', '11110'),
    'T': ('11111', '00100', '00100', '00100', '00100', '00100', '00100'),
    'U': ('10001', '10001', '10001', '10001', '10001', '10001', '01110'),
    'V': ('10001', '10001', '10001', '10001', '10001', '01010', '00100'),
    'W': ('10001', '10001', '10001', '10101', '10101', '11011', '10001'),
    'X': ('10001', '10001', '01010', '00100', '01010', '10001', '10001'),
    'Y': ('10001', '10001', '01010', '00100', '00100', '00100', '00100'),
    'Z': ('11111', '00001', '00010', '00100', '01000', '10000', '11111'),
    '0': ('01110', '10001', '10011', '10101', '11001', '10001', '01110'),
    '1': ('00100', '01100', '00100', '00100', '00100', '00100', '01110'),
    '2': ('01110', '10001', '00001', '00010', '00100', '01000', '11111'),
    '3': ('11111', '00010', '00100', '00010', '00001', '10001', '01110'),
    '4': ('00010', '00110', '01010', '10010', '11111', '00010', '00010'),
    '5': ('11111', '10000', '11110', '00001', '00001', '10001', '01110'),
    '6': ('00110', '01000', '10000', '11110', '10001', '10001', '01110'),
    '7': ('11111', '00001', '00010', '00100', '01000', '01000', '01000'),
    '8': ('01110', '10001', '10001', '01110', '10001', '10001', '01110'),
    '9': ('01110', '10001', '10001', '01111', '00001', '00010', '01100'),
    ' ': ('00000', '00000', '00000', '00000', '00000', '00000', '00000'),
    '-': ('00000', '00000', '00000', '11111', '00000', '00000', '00000'),
    '.': ('00000', '00000', '00000', '00000', '00000', '01100', '01100'),
    ':': ('00000', '01100', '01100', '00000', '01100', '01100', '00000'),
    ',': ('00000', '00000', '00000', '00000', '01100', '01100', '01000'),
    '/': ('00001', '00010', '00010', '00100', '01000', '01000', '10000'),
    '(': ('00010', '00100', '01000', '01000', '01000', '00100', '00010'),
    ')': ('01000', '00100', '00010', '00010', '00010', '00100', '01000'),
    '=': ('00000', '00000', '11111', '00000', '11111', '00000', '00000'),
    '+': ('00000', '00100', '00100', '11111', '00100', '00100', '00000'),
    '%': ('11001', '11010', '00010', '00100', '01000', '01011', '10011'),
    '?': ('01110', '10001', '00001', '00010', '00100', '00000', '00100'),
}

GLYPH_W, GLYPH_H = 5, 7


class Canvas:
    """索引色画布。颜色编号对应 PALETTE。"""

    def __init__(self, width: int, height: int, background: int = BG):
        self.w = width
        self.h = height
        self.px = bytearray([background]) * (width * height)

    def put(self, x: float, y: float, color: int) -> None:
        x, y = int(x), int(y)
        if 0 <= x < self.w and 0 <= y < self.h:
            self.px[y * self.w + x] = color

    def fill_rect(self, x0: float, y0: float, x1: float, y1: float, color: int) -> None:
        x0, x1 = sorted((max(0, int(x0)), min(self.w - 1, int(x1))))
        y0, y1 = sorted((max(0, int(y0)), min(self.h - 1, int(y1))))
        for y in range(y0, y1 + 1):
            base = y * self.w
            for x in range(x0, x1 + 1):
                self.px[base + x] = color

    def line(self, x0: float, y0: float, x1: float, y1: float, color: int, width: int = 1,
             dash: int = 0) -> None:
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        half = max(0, width // 2)
        step = 0
        while True:
            if not dash or (step // max(1, dash)) % 2 == 0:
                if width <= 1:
                    self.put(x0, y0, color)
                else:
                    for oy in range(-half, half + 1):
                        for ox in range(-half, half + 1):
                            self.put(x0 + ox, y0 + oy, color)
            step += 1
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def disc(self, cx: float, cy: float, r: float, color: int) -> None:
        cx, cy, r = int(cx), int(cy), int(r)
        for dy in range(-r, r + 1):
            span = int(math.sqrt(max(r * r - dy * dy, 0)))
            for dx in range(-span, span + 1):
                self.put(cx + dx, cy + dy, color)

    def ring(self, cx: float, cy: float, r: float, color: int, width: int = 1,
             dash_deg: float = 0.0) -> None:
        if r < 1:
            return
        steps = max(24, int(r * 2 * math.pi / 2))
        for i in range(steps):
            ang = 2 * math.pi * i / steps
            if dash_deg and (math.degrees(ang) // dash_deg) % 2 == 1:
                continue
            x = cx + r * math.cos(ang)
            y = cy + r * math.sin(ang)
            if width <= 1:
                self.put(int(x), int(y), color)
            else:
                self.disc(x, y, width / 2.0, color)

    def poly(self, points: list[tuple[float, float]], color: int) -> None:
        if len(points) < 3:
            return
        ys = [p[1] for p in points]
        for y in range(int(min(ys)), int(max(ys)) + 1):
            xs = []
            n = len(points)
            for i in range(n):
                x1, y1 = points[i]
                x2, y2 = points[(i + 1) % n]
                if (y1 <= y < y2) or (y2 <= y < y1):
                    xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
            xs.sort()
            for i in range(0, len(xs) - 1, 2):
                for x in range(int(xs[i]), int(xs[i + 1]) + 1):
                    self.put(x, y, color)

    def glyph(self, ch: str, x: float, y: float, color: int, scale: int = 1) -> None:
        x, y = int(x), int(y)
        rows = _FONT.get(ch.upper())
        if rows is None:
            return
        for row, bits in enumerate(rows):
            for col, bit in enumerate(bits):
                if bit == '1':
                    px = x + col * scale
                    py = y + row * scale
                    if scale == 1:
                        self.put(px, py, color)
                    else:
                        self.fill_rect(px, py, px + scale - 1, py + scale - 1, color)

    def text(self, x: float, y: float, content: str, color: int = TEXT, scale: int = 1) -> int:
        cursor = int(x)
        y = int(y)
        for ch in content:
            self.glyph(ch, cursor, y, color, scale)
            cursor += (GLYPH_W + 1) * scale
        return cursor

    def text_width(self, content: str, scale: int = 1) -> int:
        return len(content) * (GLYPH_W + 1) * scale


# ---------------------------------------------------------------- 视图变换
class View:
    """世界坐标（x 东、y 北，单位 m）到像素坐标的等比例映射。"""

    def __init__(self, xmin: float, xmax: float, ymin: float, ymax: float,
                 width: int, height: int, margin: int = 40):
        self.inner_w = max(1, width - 2 * margin)
        self.inner_h = max(1, height - 2 * margin)
        self.margin = margin
        span_x = max(xmax - xmin, 1e-6)
        span_y = max(ymax - ymin, 1e-6)
        self.scale = min(self.inner_w / span_x, self.inner_h / span_y)
        self.xmin = xmin
        self.ymax = ymax
        self.width = width
        self.height = height
        self.off_x = (width - span_x * self.scale) / 2.0
        self.off_y = (height - span_y * self.scale) / 2.0

    def px(self, x: float, y: float) -> tuple[float, float]:
        return (self.off_x + (x - self.xmin) * self.scale,
                self.off_y + (self.ymax - y) * self.scale)

    def length(self, meters: float) -> float:
        return meters * self.scale


# ---------------------------------------------------------------- PNG
def _png_chunk(tag: bytes, data: bytes) -> bytes:
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def write_png(path: str, canvas: Canvas, palette=PALETTE) -> None:
    raw = bytearray()
    for y in range(canvas.h):
        raw.append(0)
        base = y * canvas.w
        for x in range(base, base + canvas.w):
            raw += bytes(palette[canvas.px[x]])

    header = struct.pack('>IIBBBBB', canvas.w, canvas.h, 8, 2, 0, 0, 0)
    with open(path, 'wb') as handle:
        handle.write(b'\x89PNG\r\n\x1a\n')
        handle.write(_png_chunk(b'IHDR', header))
        handle.write(_png_chunk(b'IDAT', zlib.compress(bytes(raw), 9)))
        handle.write(_png_chunk(b'IEND', b''))


# ---------------------------------------------------------------- GIF
class _BitWriter:
    def __init__(self) -> None:
        self.out = bytearray()
        self.buf = 0
        self.cnt = 0

    def write(self, code: int, size: int) -> None:
        self.buf |= code << self.cnt
        self.cnt += size
        while self.cnt >= 8:
            self.out.append(self.buf & 0xFF)
            self.buf >>= 8
            self.cnt -= 8

    def flush(self) -> None:
        if self.cnt:
            self.out.append(self.buf & 0xFF)
            self.buf = 0
            self.cnt = 0


def _lzw(indices: bytearray, min_code_size: int) -> bytes:
    clear = 1 << min_code_size
    end = clear + 1
    writer = _BitWriter()
    code_size = min_code_size + 1
    next_code = end + 1
    table = {bytes([i]): i for i in range(clear)}

    writer.write(clear, code_size)
    prefix = bytes([indices[0]])
    for value in indices[1:]:
        candidate = prefix + bytes([value])
        if candidate in table:
            prefix = candidate
            continue
        writer.write(table[prefix], code_size)
        if next_code < 4096:
            table[candidate] = next_code
            next_code += 1
            if next_code - 1 == (1 << code_size) and code_size < 12:
                code_size += 1
        else:
            writer.write(clear, code_size)
            table = {bytes([i]): i for i in range(clear)}
            code_size = min_code_size + 1
            next_code = end + 1
        prefix = bytes([value])
    writer.write(table[prefix], code_size)
    writer.write(end, code_size)
    writer.flush()
    return bytes(writer.out)


def _blocks(data: bytes) -> bytes:
    out = bytearray()
    for i in range(0, len(data), 255):
        chunk = data[i:i + 255]
        out.append(len(chunk))
        out += chunk
    out.append(0)
    return bytes(out)


def write_gif(path: str, frames: list[Canvas], palette=PALETTE, delay: int = 8) -> None:
    w, h = frames[0].w, frames[0].h
    bits = max(3, (len(palette) - 1).bit_length())
    entries = 1 << bits

    header = b'GIF89a' + struct.pack('<HH', w, h) + bytes([0xF0 | (bits - 1), 0, 0])
    table = bytearray()
    for i in range(entries):
        table += bytes(palette[i] if i < len(palette) else (0, 0, 0))

    loop = b'\x21\xFF\x0BNETSCAPE2.0\x03\x01\x00\x00\x00'
    body = bytearray()
    for frame in frames:
        body += b'\x21\xF9\x04\x04' + struct.pack('<H', delay) + b'\x00\x00'
        body += b'\x2C' + struct.pack('<HHHH', 0, 0, w, h) + b'\x00'
        body += bytes([bits])
        body += _blocks(_lzw(frame.px, bits))

    with open(path, 'wb') as handle:
        handle.write(header + bytes(table) + loop + bytes(body) + b'\x3B')
