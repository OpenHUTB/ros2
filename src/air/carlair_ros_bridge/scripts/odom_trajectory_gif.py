#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /uav/odom 轨迹 CSV 渲染成动态 GIF，用于 PR/文档展示.

CSV 至少包含 x,y 两列；没有输入时生成一条示例“8 字”轨迹。
"""
from __future__ import annotations

import argparse
import csv
import math
import os

from PIL import Image, ImageDraw


def read_csv(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                rows.append((float(row["x"]), float(row["y"])))
            except (KeyError, ValueError):
                continue
    if not rows:
        raise ValueError("CSV 需要至少两行有效 x,y")
    return rows


def sample_trajectory(n=240):
    return [(8.0 * math.sin(t), 4.0 * math.sin(2.0 * t))
            for t in (2.0 * math.pi * i / n for i in range(n))]


def downsample(points, max_points):
    if len(points) <= max_points:
        return points
    step = max(1, len(points) // int(max_points))
    return points[::step]


def bounds(points, margin=1.0):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs) - margin, max(xs) + margin,
            min(ys) - margin, max(ys) + margin)


def render_frame(points, progress, box, size=(640, 480), label="trajectory"):
    w, h = size
    image = Image.new("RGB", size, (18, 22, 28))
    draw = ImageDraw.Draw(image)
    xmin, xmax, ymin, ymax = box
    sx = (w - 80) / max(xmax - xmin, 1e-6)
    sy = (h - 80) / max(ymax - ymin, 1e-6)
    scale = min(sx, sy)

    def xy(p):
        return (40 + (p[0] - xmin) * scale,
                h - 40 - (p[1] - ymin) * scale)

    draw.rectangle([20, 20, w - 20, h - 20], outline=(60, 70, 82))
    full = [xy(p) for p in points]
    if len(full) > 1:
        draw.line(full, fill=(55, 65, 78), width=1)
    cut = max(1, int(len(full) * min(max(progress, 0.0), 1.0)))
    if cut > 1:
        draw.line(full[:cut], fill=(80, 190, 255), width=3)
    if full:
        px, py = full[cut - 1]
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 180, 60))
    draw.text((24, 24), label, fill=(220, 230, 240))
    draw.text((24, h - 34), "x: %.1f .. %.1f m   y: %.1f .. %.1f m"
              % (xmin, xmax, ymin, ymax), fill=(140, 155, 170))
    return image


def render_gif(points, output, frames=36, fps=12, size=(640, 480)):
    points = downsample(points, 500)
    box = bounds(points)
    images = [render_frame(points, (i + 1) / frames, box, size)
              for i in range(frames)]
    output = os.path.abspath(output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    images[0].save(output, save_all=True, append_images=images[1:],
                   duration=int(1000 / max(fps, 1)), loop=0)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="odom 轨迹 GIF")
    parser.add_argument("--input", default="", help="含 x,y 的 CSV")
    parser.add_argument("--output", default="odom_trajectory.gif")
    parser.add_argument("--frames", type=int, default=36)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--demo", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    points = sample_trajectory() if args.demo or not args.input else read_csv(args.input)
    render_gif(points, args.output, args.frames, args.fps)
    print("saved", args.output, "points", len(points))


if __name__ == "__main__":
    main()
