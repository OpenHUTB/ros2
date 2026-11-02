#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把逆解的关节轨迹渲染成 GIF（需要 Pillow）。

画的是简单线框：连杆用粗线、关节用小圆、末端用空心圈、目标用十字。
投影方式是固定的斜视图，保证同一个场景每帧看得一致。

用法::

    python3 animate.py                    # 默认：7 自由度手臂画圆
    python3 animate.py --fps 15 --width 720

没有 Pillow 时会提示安装，不影响主程序。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

try:
    from PIL import Image, ImageDraw
except ImportError:  # pragma: no cover
    print("需要 Pillow：pip3 install pillow")
    raise SystemExit(1)


# ---------------------------------------------------------------- 投影
def make_projector(view, width, height, scale, center):
    """返回把 3D 点映射到屏幕坐标的函数。

    view = "xy"  俯视（适合平面臂）
    view = "xz"  斜视（适合三维手臂，绕 z 轴转 30° 后看 x-z）
    """
    a = math.radians(30.0)
    ca, sa = math.cos(a), math.sin(a)

    def project(p):
        if view == "xy":
            u, v = p[0], p[1]
        else:
            u = p[0] * ca - p[1] * sa
            v = p[2]
        return (width * 0.5 + (u - center[0]) * scale,
                height * 0.5 - (v - center[1]) * scale)

    return project


# ---------------------------------------------------------------- 渲染
BG = (250, 250, 252)
GRID = (226, 226, 232)
LINK = (40, 90, 200)
JOINT = (30, 30, 40)
EE = (215, 60, 55)
TARGET = (35, 165, 90)
TEXT = (45, 45, 50)
PATH = (185, 225, 195)     # 目标圆轨迹
TRAIL = (240, 175, 170)    # 末端拖尾


def render_frame(chain, q, target, project, width, height, caption="",
                 path=None, trail=None):
    img = Image.new("RGB", (width, height), BG)
    d = ImageDraw.Draw(img)

    # 网格
    step = 60
    for x in range(0, width, step):
        d.line([(x, 0), (x, height)], fill=GRID)
    for y in range(0, height, step):
        d.line([(0, y), (width, y)], fill=GRID)

    # 目标轨迹（整条圆，用浅绿点标出来）
    if path:
        for p in path:
            px, py = project(p)
            d.ellipse([px - 2, py - 2, px + 2, py + 2], fill=PATH)

    zs, ps, p_ee, _r = chain.frames(q)
    pts = [project(p) for p in ps] + [project(p_ee)]

    # 末端走过的拖尾
    if trail:
        for i in range(1, len(trail)):
            d.line([project(trail[i - 1]), project(trail[i])], fill=TRAIL, width=2)

    # 连杆
    for i in range(len(pts) - 1):
        d.line([pts[i], pts[i + 1]], fill=LINK, width=6)

    # 关节
    for p in pts[:-1]:
        d.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill=JOINT)

    # 目标
    if target is not None:
        tx, ty = project(target)
        d.line([(tx - 10, ty), (tx + 10, ty)], fill=TARGET, width=2)
        d.line([(tx, ty - 10), (tx, ty + 10)], fill=TARGET, width=2)
        d.ellipse([tx - 13, ty - 13, tx + 13, ty + 13], outline=TARGET, width=2)

    # 末端
    ex, ey = pts[-1]
    d.ellipse([ex - 8, ey - 8, ex + 8, ey + 8], outline=EE, width=3)

    if caption:
        d.text((14, 12), caption, fill=TEXT)
    d.text((14, height - 22),
           "blue=links  dot=joints  red=end effector  green=target  "
           "light dots=circle path", fill=(120, 120, 130))
    return img


def make_gif(out_path, frames, fps=12):
    if not frames:
        raise ValueError("没有帧")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000.0 / fps), loop=0, optimize=True)
    return out_path


# ---------------------------------------------------------------- 生成
def animate_arm_circle(out_path, n_frames=48, radius=0.12, fps=12,
                       width=720, height=520, view="xz", scale=420.0):
    """让 7 自由度手臂的末端沿一个圆走一圈，并记录关节变化。"""
    from ik_solver import solve_position_ik
    from kinematics import make_humanoid_arm

    chain = make_humanoid_arm()
    center = [0.0, 0.0, -0.45]

    project = make_projector(view, width, height, scale, (0.0, -0.45))

    # 整条圆的采样点，画成浅绿小点当背景
    circle_path = []
    for k in range(72):
        a = 2.0 * math.pi * k / 72
        circle_path.append([center[0] + radius * math.cos(a),
                            center[1],
                            center[2] + radius * math.sin(a)])

    frames = []
    q = None
    trail = []
    for i in range(n_frames):
        ang = 2.0 * math.pi * i / n_frames
        # 圆画在 x-z 平面（竖直圆），侧视图能看出是个圆；
        # 画在 x-y 平面会被侧视投影压成一条线段。
        target = [center[0] + radius * math.cos(ang),
                  center[1],
                  center[2] + radius * math.sin(ang)]
        res = solve_position_ik(chain, target, q0=q, tol=1e-5, max_iter=120)
        q = res.q
        _zs, _ps, p_ee, _r = chain.frames(q)
        trail.append(list(p_ee))
        cap = "frame %02d/%02d   |err| = %.2e m" % (i + 1, n_frames, res.error)
        frames.append(render_frame(chain, q, target, project,
                                   width, height, cap,
                                   path=circle_path, trail=trail))

    make_gif(out_path, frames, fps=fps)
    size = os.path.getsize(out_path)
    print("  已保存 %s  (%.2f MB)" % (out_path, size / 1024.0 / 1024.0))
    return out_path


def main():
    ap = argparse.ArgumentParser(description="逆运动学演示动图")
    ap.add_argument("--out", default=os.path.join(HERE, "demo_out", "ik_circle.gif"))
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--radius", type=float, default=0.12)
    args = ap.parse_args()

    print("渲染 7 自由度手臂末端画圆...")
    animate_arm_circle(args.out, fps=args.fps, width=args.width,
                       radius=args.radius)
    return 0


if __name__ == "__main__":
    sys.exit(main())
