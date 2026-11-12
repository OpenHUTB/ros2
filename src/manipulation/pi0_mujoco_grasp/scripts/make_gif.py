#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert rendered frames into stills / GIF / MP4.

Usage:
    python scripts/make_gif.py --frames /tmp/pi0_frames --out results
    python scripts/make_gif.py --frames /tmp/pi0_frames --out results --fps 30
"""
import argparse
import os
import shutil
import subprocess

from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out", default="results")
    ap.add_argument("--gif-width", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30,
                    help="录制帧率（与 main.py --fps 一致；用于 MP4/GIF 播放速度）")
    args = ap.parse_args()

    frames = sorted(f for f in os.listdir(args.frames) if f.endswith(".png"))
    frames = [os.path.join(args.frames, f) for f in frames]
    if not frames:
        raise SystemExit("no frames found")
    print("frames:", len(frames))
    os.makedirs(args.out, exist_ok=True)

    # key stills (roughly: home / grasp / lift / above plate / place / final)
    total = len(frames)
    stills = {
        "demo_home": 0,
        "demo_grasp": int(total * 0.27),
        "demo_lift": int(total * 0.45),
        "demo_above_plate": int(total * 0.65),
        "demo_place": int(total * 0.76),
        "demo_final": total - 1,
    }
    for name, idx in stills.items():
        Image.open(frames[idx]).save(os.path.join(args.out, name + ".png"))
        print("still", name, idx)

    # GIF（每 2 帧取 1 帧，播放帧率 = fps/2）
    sel = frames[::2]
    imgs = [Image.open(f).convert("RGB").resize((args.gif_width, int(args.gif_width * 0.75)))
            for f in sel]
    quantized = [im.quantize(colors=96, method=Image.MEDIANCUT) for im in imgs]
    gif = os.path.join(args.out, "demo_pick_place.gif")
    duration = int(round(2000.0 / max(1, args.fps)))
    quantized[0].save(gif, save_all=True, append_images=quantized[1:],
                      duration=duration, loop=0, optimize=True)
    print("gif:", gif, os.path.getsize(gif), "bytes")

    # optional MP4 (needs ffmpeg in PATH)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        mp4 = os.path.join(args.out, "demo_pick_place.mp4")
        subprocess.run([ffmpeg, "-y", "-framerate", str(args.fps), "-i",
                        os.path.join(args.frames, "f_%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", mp4],
                       check=False)
        print("mp4:", mp4)


if __name__ == "__main__":
    main()
