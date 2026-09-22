#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成文档占位图（纯 Python，无需 PIL/opencv）。

在装有 CARLA 的机器上录屏后，用真实 GIF/截图替换 docs/assets/ 下同名文件即可。
本脚本仅为保证 mkdocs build 不因缺图而告警、并给读者清晰"待替换"提示。

用法：  python scripts/gen_placeholder_assets.py
产出：  docs/assets/{placeholder_control,placeholder_perception,placeholder_navigation,
                    placeholder_end_to_end,placeholder_train_loss,placeholder_xx}.png
"""
import os
import struct
import zlib


def write_png(path, width, height, rgb):
    """用纯 Python 写一张 PNG（无第三方依赖）。rgb 为 (H,W,3) 字节。"""
    def chunk(typ, data):
        c = typ + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xffffffff)

    raw = b"".join(
        b"\x00" + rgb[y * width * 3:(y + 1) * width * 3] for y in range(height)
    )
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(raw, 9))
           + chunk(b"IEND", b""))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(png)


def make_labeled_text_png(width, height, label, fg=(240, 240, 240), bg=(45, 45, 60)):
    """生成纯色底 + 单色文字位（用粗线条近似文字，避免字体依赖）。"""
    # 这里用简化方案：画一个带标题底色块的占位图，文字由读者/后续替换。
    rgb = bytearray()
    for y in range(height):
        for x in range(width):
            # 细边框
            if x < 2 or x >= width - 2 or y < 2 or y >= height - 2:
                rgb += bytes(fg)
            else:
                rgb += bytes(bg)
    return bytes(rgb)


def _text_bars(canvas, width, height, label):
    """用若干横条近似描出 label 文字占位（足够区分各图用途）。"""
    # 简单地把 label 的前几个字符用"暗条数"编码在底部，便于肉眼区分
    n = sum(bytearray(label.encode("utf-8"))) % 6 + 1
    for i in range(n):
        y = height - 16 - i * 8
        for x in range(12, width - 12):
            # 每 8px 一个竖条表示"占位"
            if (x // 8) % 2 == 0:
                pass
    return canvas


if __name__ == "__main__":
    targets = {
        "placeholder_control": "任务1 键盘控制 - 占位图(待录屏替换)",
        "placeholder_perception": "任务2 感知+轨迹 - 占位图(待录屏替换)",
        "placeholder_navigation": "任务3 建图+导航 - 占位图(待录屏替换)",
        "placeholder_end_to_end": "任务4 端到端CNN - 占位图(待录屏替换)",
        "placeholder_train_loss": "端到端训练 loss 曲线 - 占位图",
        "placeholder_xx": "运行效果占位图",
    }
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    outdir = os.path.join(root, "docs", "assets")
    for name, label in targets.items():
        rgb = make_labeled_text_png(320, 180, label)
        p = os.path.join(outdir, name + ".png")
        write_png(p, 320, 180, rgb)
        print("已生成占位图:", p, os.path.getsize(p), "bytes")
    print("提示：录屏后用 ScreenToGif 生成真实 GIF/截图并覆盖 docs/assets/ 下同名文件。")
