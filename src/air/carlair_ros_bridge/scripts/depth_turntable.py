#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把深度点云渲染成旋转 GIF，用于 PR/文档展示.

输入是 .npy 的 (N,3) 点云（相机光学系：x 右、y 下、z 前），输出旋转 turntable GIF。
"""
from __future__ import annotations

import argparse
import os

import numpy as np
from PIL import Image, ImageDraw


def load_points(path):
    points = np.asarray(np.load(path), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("点云必须是 (N, 3)")
    points = points[np.isfinite(points).all(axis=1)]
    return points


def subsample(points, max_points):
    if len(points) <= max_points:
        return points
    idx = np.linspace(0, len(points) - 1, int(max_points)).astype(int)
    return points[idx]


def turntable_angles(frames):
    return np.linspace(0.0, 2.0 * np.pi, int(frames), endpoint=False)


def render_frame(points, angle, size=(640, 480)):
    w, h = size
    image = Image.new("RGB", size, (18, 22, 28))
    draw = ImageDraw.Draw(image)

    c, s = np.cos(angle), np.sin(angle)
    # 绕竖直轴旋转；z 作纵深，y 向下取反作高度
    u = points[:, 0] * c - points[:, 2] * s
    depth = points[:, 0] * s + points[:, 2] * c
    height = -points[:, 1]

    scale = min(w, h) / 16.0
    cx, cy = w / 2.0, h / 2.0 + 40.0
    px = np.clip(cx + u * scale, 0, w - 1).astype(int)
    py = np.clip(cy - height * scale, 0, h - 1).astype(int)
    d = np.clip((depth + 4.0) / 12.0, 0.0, 1.0)
    colors = np.stack([(40 + 200 * d), (90 - 40 * d), (220 - 180 * d)], axis=1)

    for i in range(len(points)):
        draw.point((int(px[i]), int(py[i])), fill=tuple(int(v) for v in colors[i]))

    draw.text((20, 16), "depth point cloud turntable", fill=(220, 230, 240))
    draw.text((20, 38), "angle %.0f deg" % np.degrees(angle), fill=(140, 155, 170))
    return image


def render_gif(points, output, frames=24, fps=8, size=(640, 480)):
    frames_list = [render_frame(points, a, size) for a in turntable_angles(frames)]
    output = os.path.abspath(output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    frames_list[0].save(
        output, save_all=True, append_images=frames_list[1:],
        duration=int(1000 / max(fps, 1)), loop=0)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="深度点云 turntable GIF")
    parser.add_argument("--input", required=True, help="(N,3) .npy 点云")
    parser.add_argument("--output", default="depth_turntable.gif")
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--max-points", type=int, default=12000)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    points = subsample(load_points(args.input), args.max_points)
    render_gif(points, args.output, args.frames, args.fps)
    print("saved", args.output, "points", len(points))


if __name__ == "__main__":
    main()
