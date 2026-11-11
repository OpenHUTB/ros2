#!/usr/bin/env python3
"""离屏渲染二连杆阻抗控制演示动图：先跑一遍仿真记录状态，再用 MuJoCo Renderer 逐帧出图。

用法（在包目录或 devel/lib/<pkg> 下执行）：
    MUJOCO_GL=egl python3 scripts/render_demo.py            # 产物写仓库 docs/img/chap8/
    python3 scripts/render_demo.py --out /tmp/impedance_img
Linux 无图形界面时加 MUJOCO_GL=egl，有桌面时用默认 GL 后端。

输出：
    <out>/impedance_demo.gif    10 s 演示动图（默认 400x225，16 fps，160 帧）
    <out>/impedance_viewer.png  稳态接触时的一帧

GIF 体积靠三个参数控制：分辨率、帧率、调色板颜色数（--colors）。默认 400x225 / 16 fps / 64 色
在 10 s 上约 330 KB；要更清晰可 --width 640 --height 360 --fps 25 --colors 128。
模型路径由 output_paths.resolve_model_path() 解析，源码树 / devel / install 三种布局都覆盖。
"""
import argparse
import os
import sys

import numpy as np
import mujoco
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from impedance_controller import ImpedanceController
from output_paths import resolve_out_dir, resolve_model_path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
KP = [80.0, 50.0]
BD = [8.0, 5.0]
X_DES = np.array([0.62, 0.0])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="渲染二连杆阻抗控制演示动图")
    ap.add_argument("--out", default=None, help="输出目录（默认仓库 docs/img/chap8，或 ~/.cache/<pkg>/output）")
    ap.add_argument("--model", default=None, help="模型 XML 路径（默认自动查找）")
    ap.add_argument("--seconds", type=float, default=10.0, help="仿真时长（秒）")
    ap.add_argument("--fps", type=int, default=16, help="GIF 帧率（默认 16）")
    ap.add_argument("--width", type=int, default=400, help="画面宽度（像素）")
    ap.add_argument("--height", type=int, default=225, help="画面高度（像素）")
    ap.add_argument("--colors", type=int, default=64, help="GIF 调色板颜色数（越小体积越小）")
    return ap.parse_args(argv)


def run_sim(model_path, seconds):
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    ctrl = ImpedanceController(Kp=KP, Bd=BD)

    steps = int(round(seconds / m.opt.timestep))
    every = max(1, int(round(0.02 / m.opt.timestep)))
    record = []
    for i in range(steps):
        q, qdot = d.qpos[:2].copy(), d.qvel[:2].copy()
        tau, F, x = ctrl.compute_torque(q, qdot, X_DES)
        d.ctrl[:] = np.clip(tau, -5, 5)
        mujoco.mj_step(m, d)
        if i % every == 0:
            fx = 0.0
            for ci in range(d.ncon):
                cf = np.zeros(6)
                mujoco.mj_contactForce(m, d, ci, cf)
                fx += cf[0]
            record.append((d.time, d.qpos[:2].copy(), fx))
    return m, d, record


def render(m, d, record, args, out_dir):
    # 按目标帧数等间隔抽帧：10 s × 16 fps = 160 帧（不能直接用 1/fps 除以 0.02，
    # 20 fps 时 0.05/0.02=2.5 会被 round 成 2，实际出 250 帧、时长只剩 8 s）
    target = max(1, int(round(args.seconds * args.fps)))
    if len(record) > target:
        idx = np.linspace(0, len(record) - 1, target).round().astype(int)
        record = [record[i] for i in idx]
    print("抽帧: %d 个采样 → %d 帧（目标 %d）" % (len(record), len(record), target))

    r = mujoco.Renderer(m, height=args.height, width=args.width)
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (0.30, 0.0, 0.12)
    cam.distance = 1.25
    cam.azimuth = 135.0
    cam.elevation = -28.0

    frames = []
    for t, qpos, fx in record:
        d.time = t
        d.qpos[:2] = qpos
        mujoco.mj_forward(m, d)
        r.update_scene(d, camera=cam)
        img = Image.fromarray(r.render()).convert("RGB")
        dr = ImageDraw.Draw(img)
        dr.rectangle([0, 0, 190, 40], fill=(0, 0, 0))
        dr.text((6, 4), "t = %5.2f s" % t, fill=(255, 255, 255))
        dr.text((6, 21), "contact F = %5.1f N" % fx, fill=(255, 210, 80))
        frames.append(img)

    # 量化到指定颜色数，体积能压到原来的三分之一左右
    pal = frames[0].quantize(colors=args.colors, method=Image.MEDIANCUT)
    frames = [f.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for f in frames]

    gif_path = os.path.join(out_dir, "impedance_demo.gif")
    frames[0].save(gif_path, save_all=True, append_images=frames[1:],
                   duration=int(round(1000.0 / args.fps)), loop=0, optimize=True)
    png_path = os.path.join(out_dir, "impedance_viewer.png")
    frames[len(frames) // 2].convert("RGB").save(png_path)
    print("frames=%d fps=%d %dx%d gif=%d B png=%d B"
          % (len(frames), args.fps, args.width, args.height,
             os.path.getsize(gif_path), os.path.getsize(png_path)))
    print("输出：", gif_path, png_path)


def main(argv=None):
    args = parse_args(argv)
    out_dir = resolve_out_dir(ROOT, args.out)
    model_path = resolve_model_path(SCRIPT_DIR, args.model)
    print("模型：", model_path)
    m, d, record = run_sim(model_path, args.seconds)
    render(m, d, record, args, out_dir)


if __name__ == "__main__":
    main()
