#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert rendered frames into stills / GIF / MP4.

Usage:
    python scripts/make_gif.py --frames /tmp/dc_frames --out results
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
    args = ap.parse_args()

    frames = sorted(os.path.join(args.frames, f) for f in os.listdir(args.frames) if f.endswith(".png"))
    if not frames:
        raise SystemExit("no frames")
    os.makedirs(args.out, exist_ok=True)
    print("frames:", len(frames))

    sel = frames[::3]  # 20fps render -> ~6.7fps GIF
    imgs = [Image.open(f).convert("RGB").resize((args.gif_width, int(args.gif_width * 0.75)))
            for f in sel]
    q = [im.quantize(colors=96, method=Image.MEDIANCUT) for im in imgs]
    gif = os.path.join(args.out, "demo_charging_nest.gif")
    q[0].save(gif, save_all=True, append_images=q[1:], duration=150, loop=0, optimize=True)
    print("gif:", gif, os.path.getsize(gif), "bytes")

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        mp4 = os.path.join(args.out, "demo_charging_nest.mp4")
        subprocess.run([ffmpeg, "-y", "-framerate", "20", "-i", os.path.join(args.frames, "f_%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", mp4],
                       check=False)
        print("mp4:", mp4)


if __name__ == "__main__":
    main()
