#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把抓到的整屏帧裁成交付资产。

为什么不自动定位轨迹：点云用 AxisColor(Z) 渲染成密集彩虹，轨迹那点纯蓝像素
会被淹没，按颜色找包围盒会误检到点云。因此用固定窗口（由观察得出），
也顺带保证动图镜头稳定。

窗口默认值基于 1280x800 的虚屏 + capture_view.rviz 的默认相机。
若你的分辨率或相机不同，用 --view/--focus 覆盖。

用法
====
    # 默认：读 /tmp/shots/s_*.png，输出到 /tmp
    python3 make_assets.py

    # 自定义分辨率与输出目录
    python3 make_assets.py --frames ~/shots \
        --view 0,20,1920,1080 --focus 600,400,1300,900 --out ~/assets
"""
import argparse
import glob
import os
import sys

from PIL import Image

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 默认值：1280x800 虚屏下 3D 视口的位置（左边 340px 是 Displays 面板）
DEF_VIEW = "340,20,1280,800"
# 在视口坐标系里的聚焦窗口（视口尺寸 940x780）。由实拍观察得出：
# 轨迹与红色路径落在约 (560~790, 420~570)，取居中的 4:3 窗口框住它。
DEF_FOCUS = "400,300,900,675"       # -> 500 x 375（4:3）


def parse_box(s):
    try:
        v = tuple(int(x) for x in s.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("格式应为 x0,y0,x1,y1，收到 %r" % s)
    if len(v) != 4 or v[2] <= v[0] or v[3] <= v[1]:
        raise argparse.ArgumentTypeError("坐标不合法: %r" % s)
    return v


def main():
    ap = argparse.ArgumentParser(description="把抓屏帧裁成静态图与 GIF")
    ap.add_argument("--frames", default="/tmp/shots",
                    help="源帧目录，默认 /tmp/shots")
    ap.add_argument("--out", default="/tmp",
                    help="输出目录，默认 /tmp")
    ap.add_argument("--view", type=parse_box, default=parse_box(DEF_VIEW),
                    help="3D 视口在整屏中的位置 x0,y0,x1,y1，默认 %s" % DEF_VIEW)
    ap.add_argument("--focus", type=parse_box, default=parse_box(DEF_FOCUS),
                    help="视口内的聚焦窗口 x0,y0,x1,y1，默认 %s" % DEF_FOCUS)
    ap.add_argument("--width", type=int, default=640,
                    help="GIF 宽度像素，默认 640")
    ap.add_argument("--stride", type=int, default=2,
                    help="每 N 帧取 1 帧做 GIF，默认 2")
    args = ap.parse_args()

    out_png = os.path.join(args.out, "rviz_planning.png")
    out_gif = os.path.join(args.out, "teleop_demo.gif")
    os.makedirs(args.out, exist_ok=True)

    frames = sorted(glob.glob(os.path.join(args.frames, "s_*.png")))
    print("源帧 %d（来自 %s）" % (len(frames), args.frames))
    if not frames:
        sys.exit("未找到 s_*.png；请先运行 capture_teleop_demo.py")

    def load(p):
        im = Image.open(p).convert("RGB").crop(args.view)
        return im.crop(args.focus)

    # 静态图：取中后段（轨迹已出、飞行球在动）
    idx = int(len(frames) * 0.55)
    shot = load(frames[idx])
    shot.save(out_png)
    print("静态图 %s %s <- %s" % (out_png, shot.size,
                                  os.path.basename(frames[idx])))

    # 动图：同一窗口，保证镜头稳定
    gif = []
    for g in frames[::args.stride]:
        im = load(g)
        r = args.width / im.width
        im = im.resize((args.width, int(im.height * r)), Image.LANCZOS)
        gif.append(im.convert("P", palette=Image.ADAPTIVE, colors=128))
    if gif:
        gif[0].save(out_gif, save_all=True, append_images=gif[1:],
                    duration=200, loop=0, optimize=True)
        print("动图 %s %d 帧 %.2f MB"
              % (out_gif, len(gif), os.path.getsize(out_gif) / 1e6))


if __name__ == "__main__":
    main()
