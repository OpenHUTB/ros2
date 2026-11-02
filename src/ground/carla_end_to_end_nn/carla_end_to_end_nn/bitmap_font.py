#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""极简位图字体：在没有 Pillow / matplotlib 的环境里给取证图写标题与坐标轴标签。

为什么需要它：本模块的取证图是**纯 numpy + zlib 手写 PNG**（见 main.py 的
`_write_png`），不依赖 Pillow / matplotlib，因此没有 `set_xlabel` 之类的接口。
评审指出「取证图没有标题/坐标轴/图例，图本身不能自证」，这里补一层最小可用
的文字绘制能力：把 5×7 点阵位图直接写进 PNG 的像素里。

位图来源：用 Pillow 的默认字体（Aileron, SIL OFL 1.1）在 14 px 下栅格化
ASCII 32..126，只保留位图数据，运行时不依赖 Pillow。

用法：
    from carla_end_to_end_nn.bitmap_font import draw_text, text_width
    draw_text(canvas, 8, 6, "training loss (MAE)", color=(40, 40, 40))
"""

import base64
import zlib

# 字高（像素）与压缩后的字形表：每行 "码位 宽度 各扫描行(hex,逗号分隔)"
_H = 15
_BLOB = (
    "eNqNV1ua6yAIfp9VZAE+iHhdTsdp9r+EAwhq2nTmfGqb2F9ELr8Uw+EP735pX4gHCAQuzeunQOKB"
    "BEnabqWkI+tGD2pw0od3eDqI3KPB8lGO6qrDpwvfLtBDddDd0zWXG0/RD3VAy9GGXj04CCQmdHom"
    "OMnNLjQXpddssutRZUGprntXqUdXTtcjP1Cnh9IN3OTU67TvR4r+CAoJl4bTMBHEMDZpDWUmDkg4"
    "0uYBNgUZ58k22rZCMou9Vmmk+HiYmCjq3LWhIEGSqLO38nqorN5+lzEPVY44TRP0KNymNl+xqr7k"
    "ukA/o4uwdeRJMFPHJvLIa6RNewszhiSv0fN0QIuHDbNsCRJFCgPb9ulQcEgr2PKZNEWeVmBQYKZG"
    "lib3BNaJjDoOpDAzO+0QPHcOy+FJlWiHSHFtTfORsqG7HNah6RknOCmY9kM1Xx1nAdlGYVlPvY4j"
    "Z2BdYb0quGzgHfDjRFPcLJ7q5uQ9vDeLty2arhlg0ZT9JXLpiA4q97zgXxlm3qsht68JuuYAiYjD"
    "xvw4QfhmDTFd1ZRQVDzAGI2SuZOIQqTRiTuqjvZwIXeHnnfq7pRlSbkBhHGId4hQovBBzK4G6Wh7"
    "ZHOf/BoDf/JzWAFezC+5qGTyfkbXQYjHer8GRjaKOjvzFu1YYeuBJ8951KYGmdEZrs9hS43i/wbP"
    "wCtw1RlN4ZPVoFeaxB8DWy5dUhw4tvdXBd9fJnuul2h+3tp7vBfLISaXzL6K1SXyieeHODxjHiuW"
    "SeFzX6aaFwvZX0YvdEclHcRQQQfSgKm3cV4WckjSxxVE67iXTaGme2iM8i4eroPmel43Up0eFIYZ"
    "FuHEft54sMJ/yzeDVvMjzSzCDvI6KByXHyte6I41EbLPzMtXrq3RYtpurL0pJt3H0GJPXElSswrk"
    "9AiuB/N/dFQBaMkwJZcDUL1Jisl40MEftKqNkaj+4IG+y9CFdamk9EkVAp+RvumqyLs12sJmudyo"
    "AGHk6zmb34lfL7CqpB9lWoFcOBS9V19bGZAg1yYLH9Pj5tpTqaFIgdumUuLkZ4oUXi89XEqer5Yu"
    "BP1SGZwDk2dqfyolW9kug0Hj4MgSFspgnNKG8aN1vUtx8Su9JnNVa5tYqxA0Ia6kAX44QG3wo15c"
    "Ujdeoxx505bFnbdyhzOGx84XfxlmL+I+7W2VAsGHZ1ZumwYv3aSnWbD6D8wKPs+K9Z6BkTDlZVva"
    "M3Ni4bgZu7ziFFlFZLhpC2NkJwfPlBrf30Kcd0PXgN+M/8fRATbD4l2oTO4ACBv2U1hZ3BH8f3wm"
    "tiNs3KrrJNn9FgOwJ5KEKghvSpJysWq4rLLYkOXerLBn06txUGs/w9btHIOgg1S8kOUfWJ8UBUD/"
    "fFZ1mAmZC0HTGJhoQetSOfHQRWF3F8pZHkIoD37E6aoAd3rgoB3RowujIWPDnn9GmMKquF3XEJjl"
    "cLNQfLUWYagsPMD90gjD/47yZe16zionX8pZOS2VlJd/Uf8ACLblwQ=="
)


def _decode():
    raw = zlib.decompress(base64.b64decode(_BLOB)).decode("utf-8")
    table = {}
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        code, width, rows = parts
        table[int(code)] = (int(width), [int(v, 16) for v in rows.split(",")])
    return table


_GLYPHS = _decode()


def text_width(text, scale=1):
    """文本在 scale 倍缩放下的像素宽度（含字间 1 px）。"""
    w = 0
    for ch in text:
        gw, _rows = _GLYPHS.get(ord(ch), (0, []))
        w += (gw + 1) * scale
    return w


def draw_text(canvas, x, y, text, color=(40, 40, 40), scale=1):
    """把文本画到 HWC uint8 画布的 (x, y)（左上角）。越界自动裁剪。"""
    h, w = canvas.shape[:2]
    cx = int(x)
    for ch in text:
        gw, rows = _GLYPHS.get(ord(ch), (0, []))
        if gw == 0:
            cx += (gw + 1) * scale
            continue
        for r, bits in enumerate(rows):
            for c in range(gw):
                if (bits >> (gw - 1 - c)) & 1:
                    for dy in range(scale):
                        for dx in range(scale):
                            yy = int(y) + r * scale + dy
                            xx = cx + c * scale + dx
                            if 0 <= yy < h and 0 <= xx < w:
                                canvas[yy, xx] = color
        cx += (gw + 1) * scale
    return canvas
