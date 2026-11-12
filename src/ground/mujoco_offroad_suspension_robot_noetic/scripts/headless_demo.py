#!/usr/bin/env python3
"""云端无头全流程：物理 → 渲染 → MP4/GIF/关键帧 → 曲线图 → 指标。

用法：
    MUJOCO_GL=egl python3 scripts/headless_demo.py --duration 24 --fps 16

产物（默认写到 docs/img/）：
    offroad_demo.mp4        全程演示
    offroad_demo.gif        轻量动图
    offroad_demo_*.png      关键帧
    offroad_demo_curves.png 姿态/悬架曲线
    offroad_demo_metrics.json / _log.csv
"""
import argparse
import json
import os
import sys

import numpy as np
import mujoco
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offroad_sim_core import OffroadSim, pose_summary, resolve_out_dir  # noqa: E402
from plot_curves import make_curves  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ------------------------------------------------------------------ 相机
def make_camera(kind, s, phase=0.0):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    if kind == "wide":
        cam.lookat[:] = [s["x"] + 0.6, s["y"], s["z"] - 0.10]
        cam.distance = 4.6
        cam.azimuth = 106
        cam.elevation = -15
    elif kind == "rig":          # 侧前方近景：看悬架与轮胎
        cam.lookat[:] = [s["x"], s["y"], s["z"] + 0.01]
        cam.distance = 2.15
        cam.azimuth = 124
        cam.elevation = -11
    else:                        # 车后侧上方：看整车与地形
        cam.lookat[:] = [s["x"], s["y"], s["z"] + 0.06]
        cam.distance = 3.3
        cam.azimuth = 152
        cam.elevation = -20
    return cam


def write_mp4(frames, path, fps, crf=23):
    """用系统 ffmpeg 直接编码 RGB 帧序列，不依赖 imageio 的插件。"""
    import subprocess
    h, w, _ = frames[0].shape
    if (h % 2) or (w % 2):
        raise ValueError(f"yuv420p 需要偶数尺寸，当前 {w}x{h}")
    cmd = ["ffmpeg", "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fps),
           "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium",
           "-crf", str(crf), "-pix_fmt", "yuv420p", "-movflags", "+faststart", path]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(f.tobytes())
    proc.stdin.close()
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg exit {proc.returncode}")


def caption(img, text):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, im.height - 17, im.width, im.height], fill=(0, 0, 0, 130))
    d.text((6, im.height - 15), text, fill=(238, 238, 238, 255))
    return im


# 曲线图：与纯物理出图共用 scripts/plot_curves.py，版式一致（见文件头的 import）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=24.0)
    ap.add_argument("--fps", type=int, default=16)
    ap.add_argument("--width", type=int, default=704)
    ap.add_argument("--height", type=int, default=396)
    ap.add_argument("--outdir", default=None,
                    help="输出目录（默认源码树 docs/img，装到工作空间后退到 ~/.cache/<pkg>/output）")
    ap.add_argument("--tilt-assist", type=float, default=0.0)
    ap.add_argument("--start", type=float, nargs=2, default=(-2.0, -0.5))
    ap.add_argument("--target", type=float, nargs=2, default=(18.0, 2.0))
    ap.add_argument("--tag", default="offroad_demo")
    ap.add_argument("--gif-width", type=int, default=420)
    ap.add_argument("--gif-fps", type=int, default=10)
    ap.add_argument("--gif-colors", type=int, default=64)
    args = ap.parse_args()
    args.outdir = resolve_out_dir(args.outdir)
    os.makedirs(args.outdir, exist_ok=True)

    sim = OffroadSim(target=tuple(args.target), start=tuple(args.start),
                     tilt_assist_gain=args.tilt_assist)
    renderer = mujoco.Renderer(sim.model, args.height, args.width)

    frames_rgb = []          # 全分辨率，供 mp4
    samples = []
    frame_dt = 1.0 / args.fps

    # 开场静止远景
    n_intro = int(0.9 * args.fps)
    for _ in range(n_intro):
        cam = make_camera("wide", sim.state())
        renderer.update_scene(sim.data, camera=cam)
        frames_rgb.append(renderer.render().copy())

    next_frame = sim.data.time
    t_end = sim.data.time + args.duration
    still_marks = [(0.04, "start", "wide"), (0.45, "mid", "rig"), (0.97, "end", "chase")]
    stills_done = set()
    while sim.data.time < t_end:
        cmd = sim.step()
        if sim.data.time >= next_frame:
            s = sim.state()
            s["cmd"] = cmd
            frac = sim.data.time / t_end
            kind = "rig" if frac < 0.72 else "chase"
            cam = make_camera(kind, s, frac)
            renderer.update_scene(sim.data, camera=cam)
            rgb = renderer.render().copy()
            frames_rgb.append(rgb)
            samples.append(s)
            for mk_frac, mk_tag, _ in still_marks:
                if mk_tag not in stills_done and frac >= mk_frac:
                    stills_done.add(mk_tag)
                    Image.fromarray(rgb).save(
                        os.path.join(args.outdir, f"{args.tag}_{mk_tag}.png"))
                    print(f"still {mk_tag}: t={s['t']:.1f}s")
            next_frame += frame_dt

    metrics = pose_summary(samples, sim.target)
    metrics["settings"] = {
        "duration_s": args.duration,
        "fps": args.fps,
        "size": [args.width, args.height],
        "tilt_assist_gain": args.tilt_assist,
        "terrain": sim.terrain_info,
        "start": list(args.start),
        "target": list(args.target),
    }

    # ---- MP4
    mp4 = os.path.join(args.outdir, f"{args.tag}.mp4")
    try:
        write_mp4(frames_rgb, mp4, args.fps)
        print(f"MP4: {mp4}  ({os.path.getsize(mp4) / 1e6:.2f} MB, {len(frames_rgb)} frames)")
    except Exception as e:  # noqa: BLE001
        print("MP4 写入失败:", e)

    # ---- GIF（降分辨率抽帧）
    step = max(1, int(round(args.fps / args.gif_fps)))
    gw = args.gif_width
    gh = int(round(gw * args.height / args.width / 2)) * 2
    gif_frames = []
    for i in range(0, len(frames_rgb), step):
        img = Image.fromarray(frames_rgb[i]).resize((gw, gh), Image.LANCZOS)
        si = i - n_intro
        if 0 <= si < len(samples):
            s = samples[si]
            d = ImageDraw.Draw(img, "RGBA")
            d.rectangle([0, gh - 14, gw, gh], fill=(0, 0, 0, 130))
            d.text((4, gh - 12),
                   "t=%4.1fs v=%.2f roll=%+.1f pitch=%+.1f" % (
                       s["t"], s["v_body"], np.degrees(s["roll"]), np.degrees(s["pitch"])),
                   fill=(235, 235, 235, 255))
        gif_frames.append(img)
    # 全片共用一套调色板并关闭抖动：逐帧各自 ADAPTIVE 会让每帧都存完整像素（3.8 MB），
    # 共用调色板后帧间只存差异，体积降一个量级（1.0 MB）
    pal = gif_frames[0].convert("P", palette=Image.ADAPTIVE,
                                colors=args.gif_colors, dither=Image.Dither.NONE)
    gif_frames = [pal] + [im.convert("P", palette=pal, dither=Image.Dither.NONE)
                          for im in gif_frames[1:]]
    gif = os.path.join(args.outdir, f"{args.tag}.gif")
    gif_frames[0].save(gif, save_all=True, append_images=gif_frames[1:],
                       duration=int(round(1000.0 * step / args.fps)),
                       loop=0, optimize=True, disposal=2)
    print(f"GIF: {gif}  ({len(gif_frames)} frames, {os.path.getsize(gif) / 1e6:.2f} MB)")

    with open(os.path.join(args.outdir, f"{args.tag}_metrics.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v)
                      for k, v in metrics.items() if k != "settings"},
                     ensure_ascii=False, indent=1))

    csv_path = os.path.join(args.outdir, f"{args.tag}_log.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("t,x,y,z,roll_deg,pitch_deg,yaw_deg,v_body,sus_fl,sus_fr,sus_rl,sus_rr\n")
        for s in samples:
            f.write("%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f\n" % (
                s["t"], s["x"], s["y"], s["z"], np.degrees(s["roll"]), np.degrees(s["pitch"]),
                np.degrees(s["yaw"]), s["v_body"], *s["sus"]))
    print(f"CSV: {csv_path}")

    make_curves(samples, sim.target,
                os.path.join(args.outdir, f"{args.tag}_curves.png"))


if __name__ == "__main__":
    main()
