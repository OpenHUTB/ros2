#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把终端日志 / 测试输出渲染成 PNG，用于在 PR 里提交"真实运行截图"。

仓库约定：用**截图**而不是拍照，图片用 .png 且小于 3 MB。很多同学拿手机拍屏幕，
或者贴 GitHub 外链图片（PR 里不可复现）——这个工具让你直接把**本机真实输出**
转成一张规整的 PNG 提交进仓库。

用法
----
::

    # 1) 先把真实输出存成文本
    python3 tests/test_gesture_control_local.py 2>&1 | tee /tmp/local_test.log
    # 2) 再转成图片（工具会自动放在 docs/img 下，符合仓库约定）
    python3 tools/log_to_png.py /tmp/local_test.log docs/img/air/gesture_control_drone/local_test.png \\
        --title "python3 tests/test_gesture_control_local.py   ->   212 次断言全通过"

    # AirSim 真机运行日志同理
    roslaunch gesture_control_drone main.launch airsim_ip:=192.168.80.1 2>&1 | tee /tmp/run.log
    python3 tools/log_to_png.py /tmp/run.log docs/img/air/gesture_control_drone/run.png \\
        --title "roslaunch gesture_control_drone main.launch (AirSim Blocks, ENU)"

参数
----
``--title``    图片顶部标题栏文字
``--max-lines``最多渲染多少行（超出的头尾各取一半，中间显示省略提示）
``--font-size``字号（默认 14）
``--max-width``图片最大像素宽（默认 1400，超过会自动缩小字号）
"""
from __future__ import annotations

import argparse
import os
import sys

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:                      # pragma: no cover
    sys.exit("需要 Pillow：pip3 install Pillow")

# 优先选带中文字形、且等宽感较好的字体
FONT_CANDIDATES = (
    "C:/Windows/Fonts/msyh.ttc",             # 微软雅黑（含中文）
    "C:/Windows/Fonts/simhei.ttf",           # 黑体
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "DejaVuSansMono.ttf",
)
BG = (30, 30, 30)
BAR = (58, 58, 58)
FG = (220, 220, 220)
GREEN = (106, 220, 120)
RED = (240, 110, 110)
YELLOW = (240, 200, 100)
CYAN = (120, 200, 240)
PAD = 16

#: 等宽字体里没有这些 emoji 字形，渲染成方块；替换成 ASCII 以免出现豆腐块
_EMOJI_MAP = {
    "\u2705": "[OK]", "\u274c": "[X]", "\u26a0\ufe0f": "[!]",
    "\u26a0": "[!]", "\u2b50": "*", "\u2192": "->", "\u2026": "...",
    "\u2713": "[v]", "\u2717": "[x]",
}


def sanitize(text):
    for k, v in _EMOJI_MAP.items():
        text = text.replace(k, v)
    out = []
    for ch in text:
        cp = ord(ch)
        # 保留 ASCII / 常用符号 / CJK / 全角标点，其余（emoji 等）替换掉
        if cp < 0x2500 or 0x3000 <= cp <= 0x9FFF or 0xFF00 <= cp <= 0xFFEF:
            out.append(ch)
        else:
            out.append("?")
    return "".join(out)


def pick_font(size):
    for path in FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size), path
        except (OSError, IOError):
            continue
    return ImageFont.load_default(), "(default)"


def line_color(text):
    if text.startswith("[通过]"):
        return GREEN
    if text.startswith("[失败]"):
        return RED
    if text.startswith("$") or text.startswith("#"):
        return CYAN
    if any(k in text for k in ("Error", "错误", "失败", "Traceback", "⚠")):
        return YELLOW
    return FG


def wrap_line(text, font, max_px):
    """按像素宽度贪心折行（中文按字符切即可）。"""
    if not text:
        return [""]
    out, cur = [], ""
    for ch in text:
        if font.getlength(cur + ch) <= max_px:
            cur += ch
        else:
            out.append(cur)
            cur = ch
    if cur:
        out.append(cur)
    return out


def render(log_path, out_path, title=None, font_size=14, max_width=1400,
           max_lines=140):
    with open(log_path, encoding="utf-8", errors="replace") as fh:
        raw = fh.read().replace("\r\n", "\n").replace("\r", "\n")
    lines = [ln.rstrip() for ln in raw.split("\n")]
    while lines and not lines[-1]:
        lines.pop()
    if len(lines) > max_lines:
        head = max_lines // 2
        tail = max_lines - head - 1
        lines = lines[:head] + ["…（略去 %d 行）…" % (len(lines) - head - tail)] + \
            lines[-tail:]

    font, font_path = pick_font(font_size)
    title = title or os.path.basename(log_path)
    text_max_px = max_width - 2 * PAD

    wrapped = []
    for ln in lines:
        for sub in wrap_line(sanitize(ln), font, text_max_px):
            wrapped.append((sub, line_color(ln)))

    line_h = int(font_size * 1.55)
    title_h = int(font_size * 2.2)
    width = max_width
    height = title_h + PAD + line_h * len(wrapped) + PAD

    img = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, width, title_h], fill=BAR)
    for i, cx in enumerate((18, 38, 58)):            # 三个"窗口按钮"
        draw.ellipse([cx - 6, title_h // 2 - 6, cx + 6, title_h // 2 + 6],
                     fill=(200, 90, 90) if i == 0 else
                          (210, 180, 90) if i == 1 else (110, 190, 110))
    draw.text((80, (title_h - font_size) // 2 - 2), title, font=font, fill=FG)

    y = title_h + PAD // 2
    for text, color in wrapped:
        draw.text((PAD, y), text, font=font, fill=color)
        y += line_h

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    img.save(out_path, "PNG", optimize=True)
    size = os.path.getsize(out_path)
    print("已生成 %s  (%d×%d, %.0f KB, 字体 %s)"
          % (out_path, width, height, size / 1024.0, os.path.basename(font_path)))
    if size >= 3 * 1024 * 1024:
        print("⚠ 超过 3 MB，请减小 --max-lines 或 --font-size")
    return out_path


def main():
    ap = argparse.ArgumentParser(description="日志/终端输出 -> PNG 截图")
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--title", default=None)
    ap.add_argument("--font-size", type=int, default=14)
    ap.add_argument("--max-width", type=int, default=1400)
    ap.add_argument("--max-lines", type=int, default=140)
    args = ap.parse_args()
    render(args.log, args.out, title=args.title, font_size=args.font_size,
           max_width=args.max_width, max_lines=args.max_lines)
    return 0


if __name__ == "__main__":
    sys.exit(main())
