#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把闭环步态渲染成行走动画（GIF）。

需要 Pillow：``pip install pillow``。核心分析脚本不依赖它，只有出图这一步用。

画的是一个侧视火柴人：质心处画躯干，两条腿从髋关节连到脚，地面上标出当前
支撑区间和 ZMP 位置，左上角显示时间与稳定裕度。
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from dcm import DEFAULT_GAIN, track  # noqa: E402
from gait import build_symmetric_plan  # noqa: E402
from lipm import LIPMParams  # noqa: E402

try:
    from PIL import Image, ImageDraw
except ImportError:  # pragma: no cover
    print("需要 Pillow：pip install pillow")
    raise SystemExit(1)

# ---------------------------------------------------------------- 画布参数
W, H = 760, 460
GROUND_Y = 360
SCALE = 260.0            # 像素 / 米
BODY_HEIGHT = 0.80       # 质心高度（与 LIPMParams.z_c 一致）
THIGH = 0.42             # 大腿长（视觉用）
FOOT_W, FOOT_H = 0.24, 0.055

BG = (248, 248, 250)
GROUND = (110, 110, 120)
LEG_STANCE = (40, 110, 220)
LEG_SWING = (170, 190, 220)
BODY = (60, 60, 70)
SUPPORT = (210, 235, 210)
ZMP = (215, 60, 55)
TEXT = (40, 40, 45)


def _steps_of_side(plan, side):
    return [s for s in plan.steps if s.side == side]


def _foot_pose(plan, side, t):
    """返回这只脚在 t 时刻的 (位置, 抬起高度, 是否支撑)。"""
    seq = _steps_of_side(plan, side)
    if not seq:
        return 0.0, 0.0, False

    # 在支撑区间内
    for s in seq:
        if s.t_start <= t <= s.t_end:
            return s.center, 0.0, True

    # 摆动：找前后两个落脚点
    prev = None
    for s in seq:
        if s.t_end < t:
            prev = s
        elif s.t_start > t and prev is not None:
            span = s.t_start - prev.t_end
            if span <= 0:
                return s.center, 0.0, False
            u = (t - prev.t_end) / span
            u = max(0.0, min(1.0, u))
            x = prev.center + (s.center - prev.center) * u
            lift = 0.08 * 4.0 * u * (1.0 - u)      # 抛物线抬脚
            return x, lift, False

    # 边界外
    if t < seq[0].t_start:
        return seq[0].center, 0.0, False
    return seq[-1].center, 0.0, False


def _to_px(x, z, cam):
    """世界坐标 (x, z) → 屏幕像素。cam 是镜头中心的 x。"""
    px = W * 0.5 + (x - cam) * SCALE
    py = GROUND_Y - z * SCALE
    return px, py


def render_frame(plan, traj, i, cam, zmp_x=None):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    t = traj["time"][i]

    # 地面
    d.line([(0, GROUND_Y), (W, GROUND_Y)], fill=GROUND, width=2)

    # 支撑区间
    lo, hi = traj["poly_lo"][i], traj["poly_hi"][i]
    x0, _ = _to_px(lo, 0.0, cam)
    x1, _ = _to_px(hi, 0.0, cam)
    if x1 > x0:
        d.rectangle([x0, GROUND_Y + 1, x1, GROUND_Y + 14], fill=SUPPORT)

    # 两只脚
    poses = {}
    for side in ("L", "R"):
        fx, lift, stance = _foot_pose(plan, side, t)
        poses[side] = (fx, lift, stance)
        fx0, _ = _to_px(fx - FOOT_W / 2, 0.0, cam)
        fx1, _ = _to_px(fx + FOOT_W / 2, 0.0, cam)
        _, fy_top = _to_px(fx, lift + FOOT_H, cam)      # z 越高 → y 越小
        _, fy_bot = _to_px(fx, lift, cam)
        d.rectangle([fx0, fy_top, fx1, fy_bot],
                    fill=(70, 130, 60) if stance else (150, 170, 150))

    # 质心与髋
    com_x = traj["com_x"][i]
    hip_px, hip_py = _to_px(com_x, BODY_HEIGHT * 0.86, cam)

    # 腿：髋 → 脚
    for side in ("L", "R"):
        fx, lift, stance = poses[side]
        ax, ay = _to_px(fx, lift + FOOT_H, cam)
        # 用两段折线近似膝盖
        mx = (hip_px + ax) * 0.5
        my = (hip_py + ay) * 0.5 + THIGH * SCALE * 0.12
        d.line([(hip_px, hip_py), (mx, my)], fill=LEG_STANCE if stance else LEG_SWING, width=7)
        d.line([(mx, my), (ax, ay)], fill=LEG_STANCE if stance else LEG_SWING, width=7)

    # 躯干
    head_y = hip_py - 46
    d.line([(hip_px, hip_py), (hip_px, head_y + 20)], fill=BODY, width=9)
    d.ellipse([hip_px - 13, head_y - 13, hip_px + 13, head_y + 13], fill=BODY)

    # 质心标记
    cxp, cyp = _to_px(com_x, BODY_HEIGHT, cam)
    d.ellipse([cxp - 5, cyp - 5, cxp + 5, cyp + 5], fill=(235, 150, 30))

    # ZMP
    if zmp_x is not None:
        zx, _ = _to_px(zmp_x, 0.0, cam)
        d.line([(zx, GROUND_Y - 12), (zx, GROUND_Y + 14)], fill=ZMP, width=3)
        d.ellipse([zx - 6, GROUND_Y - 6, zx + 6, GROUND_Y + 6], outline=ZMP, width=3)

    # 文字
    d.text((14, 12), "t = %.2f s" % t, fill=TEXT)
    d.text((14, 30), "com x = %+.3f m" % com_x, fill=TEXT)
    d.text((14, 48), "margin = %+.3f m" % traj["margin"][i], fill=TEXT)
    d.text((14, 66), "zmp = %+.3f m" % traj["zmp_actual"][i], fill=TEXT)
    d.text((14, H - 22), "blue legs = stance,  red mark = ZMP", fill=(120, 120, 130))
    return img


def make_gif(out_path, t_from=None, t_to=None, fps=25, gain=DEFAULT_GAIN):
    params = LIPMParams(z_c=0.8)
    plan = build_symmetric_plan(n_steps=14, step_length=0.35, step_time=0.5, duty=0.6)
    v_avg = 0.35 / 0.5
    traj = track(plan, params, 0.0, 0.0, v_avg, dt=0.001, gain=gain)

    ts = traj["time"]
    i0 = 0 if t_from is None else max(0, int(t_from / 0.001))
    i1 = len(ts) - 1 if t_to is None else min(len(ts) - 1, int(t_to / 0.001))

    stride = max(1, int(round((1.0 / fps) / 0.001)))
    idx = list(range(i0, i1, stride))
    print("  帧数: %d  (fps=%d, %.2f~%.2f s)" % (len(idx), fps, ts[i0], ts[i1]))

    frames = []
    for n, i in enumerate(idx):
        # 镜头跟随质心，稍微提前一点，看得见下一步落脚
        cam = traj["com_x"][i] + 0.18
        frames.append(render_frame(plan, traj, i, cam, zmp_x=traj["zmp_actual"][i]))
        if n % 20 == 0:
            print("    渲染 %d/%d" % (n, len(idx)))

    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000.0 / fps), loop=0, optimize=True)
    size = os.path.getsize(out_path)
    print("  已保存 %s  (%.2f MB)" % (out_path, size / 1024.0 / 1024.0))
    return out_path


def main():
    # 与 main.py 一致：默认写到包根的 demo_out/，不是 scripts/ 下
    default = os.path.normpath(os.path.join(HERE, os.pardir, "demo_out"))
    ap = argparse.ArgumentParser(description="渲染双足行走动图（GIF）")
    ap.add_argument("--outdir", default=default,
                    help="输出目录（默认包根 demo_out/）")
    args, _unknown = ap.parse_known_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)
    print("渲染行走动画...")
    make_gif(os.path.join(args.outdir, "walking.gif"), t_from=0.5, t_to=4.5, fps=25)
    return 0


if __name__ == "__main__":
    sys.exit(main())
