#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 screenshots/ 里的帧合成 GIF。

需要 Pillow：``pip3 install pillow``

用法::

    python3 make_gif.py                    # 用默认目录
    python3 make_gif.py --fps 10 --width 640
    python3 make_gif.py --start 10 --end 40
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    print("需要 Pillow：pip3 install pillow")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="把 CARLA 截图合成 GIF")
    ap.add_argument("--dir", default=os.path.join(HERE, "screenshots"))
    ap.add_argument("--out", default=os.path.join(HERE, "screenshots", "drive.gif"))
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=0, help="0 表示到最后")
    ap.add_argument("--max-frames", type=int, default=60)
    args = ap.parse_args()

    if not os.path.isdir(args.dir):
        print("目录不存在: %s" % args.dir)
        return 1

    files = sorted(f for f in os.listdir(args.dir)
                   if f.startswith("frame_") and f.endswith(".png"))
    if not files:
        print("没有找到 frame_*.png，先跑 main.py --carla")
        return 1

    end = args.end if args.end > 0 else len(files)
    files = files[args.start:end]

    # 均匀抽帧，避免 GIF 太大
    if len(files) > args.max_frames:
        step = len(files) / float(args.max_frames)
        files = [files[int(i * step)] for i in range(args.max_frames)]

    print("用 %d 帧，fps=%d，宽度 %d" % (len(files), args.fps, args.width))

    frames = []
    for i, name in enumerate(files):
        path = os.path.join(args.dir, name)
        im = Image.open(path).convert("RGB")
        w, h = im.size
        if w != args.width:
            im = im.resize((args.width, int(h * args.width / float(w))),
                           Image.LANCZOS)
        frames.append(im)
        if i % 10 == 0:
            print("  读入 %d/%d" % (i, len(files)))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    frames[0].save(args.out, save_all=True, append_images=frames[1:],
                   duration=int(1000.0 / args.fps), loop=0, optimize=True)

    size = os.path.getsize(args.out)
    print("已保存 %s  (%.2f MB)" % (args.out, size / 1024.0 / 1024.0))
    if size > 10 * 1024 * 1024:
        print("⚠ 超过 10 MB，仓库限制是 GIF < 10 MB。减小 --width 或 --max-frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
