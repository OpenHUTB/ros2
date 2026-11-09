#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道线检测可视化演示：生成带检测结果的图片 + 打印量化指标。

用途
----
用于作业验证与截图。**不需要 CARLA、不需要 ROS、不需要 OpenCV、不需要联网**，
只用 numpy 就能跑（虚拟机里已装）。

它做三件事：
  1. 生成一张车道线位置**已知**的合成图像，因此可以算出真实的测量误差
  2. 跑完整检测流程，把拟合出的车道线**换算回原图坐标**并画出来
  3. 在终端打印量化指标，并保存图片供截图

运行::

    cd ~/ros2/src/ground/carla_lane_keeping
    python3 tests/demo_lane_detection.py

输出（默认写到包目录下的 demo_out/）::

    demo_1.png / demo_1.ppm          场景 1 的检测结果叠加图
    demo_1_mask.png / .ppm           场景 1 的鸟瞰二值掩码
    ...                              场景 2、3 同理
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from lane_detection import (  # noqa: E402
    LaneDetector,
    LaneFitConfig,
    make_synthetic_lane_image,
    warp_horizontal_scale,
)

WIDTH, HEIGHT = 800, 600


# ---------------------------------------------------------------------------
# 图像保存（不依赖第三方库）
# ---------------------------------------------------------------------------


def save_ppm(path, arr):
    """把 uint8 数组写成 PPM（P6 彩色 / P5 灰度）。"""
    a = np.asarray(arr)
    if a.ndim == 3:
        h, w, _ = a.shape
        with open(path, "wb") as fh:
            fh.write(f"P6\n{w} {h}\n255\n".encode())
            fh.write(a.astype(np.uint8).tobytes())
    else:
        h, w = a.shape
        with open(path, "wb") as fh:
            fh.write(f"P5\n{w} {h}\n255\n".encode())
            fh.write(a.astype(np.uint8).tobytes())


def save_image(path, arr):
    """优先存 PNG（需要 Pillow），没有则退回 PPM。返回实际写入的路径。"""
    try:
        from PIL import Image  # noqa: PLC0415

        Image.fromarray(np.asarray(arr, dtype=np.uint8)).save(path)
        return path
    except ImportError:
        ppm_path = os.path.splitext(path)[0] + ".ppm"
        save_ppm(ppm_path, arr)
        return ppm_path


# ---------------------------------------------------------------------------
# 可视化：把鸟瞰图的拟合结果换算回原图坐标
# ---------------------------------------------------------------------------


def draw_line(img, p0, p1, color, thickness=3):
    """在图像上画一条粗线段（纯 numpy 实现）。"""
    x0, y0 = p0
    x1, y1 = p1
    steps = int(max(abs(x1 - x0), abs(y1 - y0)))
    if steps <= 0:
        return
    half = max(0, thickness // 2)
    for i in range(steps + 1):
        t = i / steps
        x = int(round(x0 + (x1 - x0) * t))
        y = int(round(y0 + (y1 - y0) * t))
        ya, yb = max(0, y - half), min(img.shape[0], y + half + 1)
        xa, xb = max(0, x - half), min(img.shape[1], x + half + 1)
        if ya < yb and xa < xb:
            img[ya:yb, xa:xb] = color


def draw_lanes_on_source(image_bgr, detector, left_fit, right_fit,
                         color_left=(255, 64, 64), color_right=(64, 64, 255),
                         color_center=(64, 255, 64)):
    """把鸟瞰图里的车道线拟合结果画回**原图坐标**。

    关键点：鸟瞰图的 (u, v) 必须先经单应矩阵的逆映射回原图像素，画出来才与
    原图中的车道线重合。直接把鸟瞰坐标当原图画会明显错位。
    """
    out = np.array(image_bgr, dtype=np.uint8, copy=True)
    h, w = out.shape[0], out.shape[1]
    h_inv = np.linalg.inv(detector._h)

    def warped_to_source(u_w, v_w):
        p = h_inv @ np.array([u_w, v_w, 1.0])
        if abs(p[2]) < 1e-9:
            return None
        return int(round(p[0] / p[2])), int(round(p[1] / p[2]))

    def draw_poly(fit, color, thickness):
        if fit is None:
            return
        prev = None
        for v_w in range(0, h, 2):
            u_w = float(np.polyval(fit, v_w))
            pt = warped_to_source(u_w, v_w)
            if pt is None:
                prev = None
                continue
            x, y = pt
            if not (0 <= x < w and 0 <= y < h):
                prev = None
                continue
            if prev is not None:
                draw_line(out, prev, (x, y), color, thickness)
            prev = (x, y)

    draw_poly(left_fit, color_left, 3)
    draw_poly(right_fit, color_right, 3)
    if left_fit is not None and right_fit is not None:
        center_fit = 0.5 * (np.asarray(left_fit) + np.asarray(right_fit))
        draw_poly(center_fit, color_center, 2)
    return out


def draw_crosshair(img, u, v, color=(0, 255, 255), size=14, thickness=2):
    """在 (u, v) 处画十字，用于标记检出的车道中心。"""
    h, w = img.shape[0], img.shape[1]
    u, v = int(round(u)), int(round(v))
    for x in range(max(0, u - size), min(w, u + size + 1)):
        for dy in range(-(thickness // 2), thickness // 2 + 1):
            y = v + dy
            if 0 <= y < h:
                img[y, x] = color
    for y in range(max(0, v - size), min(h, v + size + 1)):
        for dx in range(-(thickness // 2), thickness // 2 + 1):
            x = u + dx
            if 0 <= x < w:
                img[y, x] = color


def ascii_view(mask, cols=74, rows=16):
    """把布尔掩码转成 ASCII 图，便于在终端直接看到结果（截图友好）。"""
    h, w = mask.shape
    lines = []
    for r in range(rows):
        v0, v1 = int(r * h / rows), int((r + 1) * h / rows)
        line = []
        for c in range(cols):
            u0, u1 = int(c * w / cols), int((c + 1) * w / cols)
            line.append("#" if mask[v0:v1, u0:u1].any() else ".")
        lines.append("".join(line))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


def run_scenario(title, index, cfg, outdir, image_kwargs):
    """跑一个场景：生成图 -> 检测 -> 出图 -> 打印指标。返回测量误差（px）或 None。"""
    image = make_synthetic_lane_image(width=WIDTH, height=HEIGHT, **image_kwargs)
    detector = LaneDetector(cfg)
    res = detector.detect(image)

    left_u = image_kwargs.get("left_x_bottom", 250.0)
    right_u = image_kwargs.get("right_x_bottom", 550.0)
    true_center_src = 0.5 * (left_u + right_u)

    print("=" * 74)
    print(f"场景 {index}：{title}")
    print("=" * 74)
    print(f"  图像尺寸            : {WIDTH} x {HEIGHT}")
    print(f"  真值车道中心        : {true_center_src:.1f} px（原图底行，解析给定）")

    if not res.detected:
        print(f"  检测结果            : 未检出两侧 "
              f"(左 {res.left_pixels} px / 右 {res.right_pixels} px)")
        print()
        return None

    # 公平比较：把真值换算到「检测器实际求值的那一行」与「鸟瞰尺度」
    scale = warp_horizontal_scale(detector._h, WIDTH, HEIGHT, res.v_eval)
    true_center_warped = WIDTH / 2.0 + (true_center_src - WIDTH / 2.0) * scale
    measured_center = WIDTH / 2.0 - res.lane_center_px
    err_px = measured_center - true_center_warped

    lane_width_m = (float(np.polyval(res.right_fit, res.v_eval))
                    - float(np.polyval(res.left_fit, res.v_eval))) * cfg.meters_per_pixel_x

    print(f"  检出两侧车道线      : 是")
    print(f"  左/右车道像素数     : {res.left_pixels} / {res.right_pixels}")
    print(f"  置信度              : {res.confidence:.2f}")
    print(f"  求值行 v            : {res.v_eval:.0f}（鸟瞰图坐标）")
    print(f"  车道宽度            : {lane_width_m:.2f} m")
    print(f"  横向偏差            : {res.lateral_offset_m:+.3f} m"
          f"   （正 = 车辆偏在车道中心右侧）")
    print(f"  航向偏差            : {math.degrees(res.heading_error_rad):+.2f} deg")
    curv = res.curvature_m
    print(f"  曲率半径            : "
          f"{'直道（无穷大）' if np.isinf(curv) else f'{curv:.1f} m'}")
    print(f"  车道中心测量值      : {measured_center:.1f} px（鸟瞰尺度）")
    print(f"  换算后的真值        : {true_center_warped:.1f} px")
    print(f"  >>> 测量误差        : {err_px:+.1f} px"
          f"  =  {err_px * cfg.meters_per_pixel_x:+.3f} m")

    # ---- 出图 ----
    overlay = draw_lanes_on_source(image, detector, res.left_fit, res.right_fit)
    u_l = float(np.polyval(res.left_fit, res.v_eval))
    u_r = float(np.polyval(res.right_fit, res.v_eval))
    draw_crosshair(overlay, 0.5 * (u_l + u_r), res.v_eval)

    base = os.path.join(outdir, f"demo_{index}")
    saved_overlay = save_image(base + ".png", overlay[:, :, ::-1])  # BGR -> RGB
    mask = detector.warped_mask(image)
    saved_mask = save_image(base + "_mask.png", mask.astype(np.uint8) * 255)

    print()
    print(f"  已保存检测结果图    : {saved_overlay}")
    print(f"  已保存鸟瞰掩码      : {saved_mask}")
    print()
    print("  --- 鸟瞰二值掩码（# = 车道像素）---")
    print(ascii_view(mask))
    print()
    return err_px


def main():
    ap = argparse.ArgumentParser(description="车道线检测可视化演示")
    ap.add_argument("--outdir", default=os.path.join(HERE, "..", "demo_out"),
                    help="图片输出目录")
    args = ap.parse_args()

    outdir = os.path.abspath(args.outdir)
    os.makedirs(outdir, exist_ok=True)

    print()
    print("=" * 74)
    print("车道线检测演示（合成图像，车道位置解析给定，故可计算真实测量误差）")
    print("=" * 74)
    print(f"输出目录: {outdir}")
    print()

    cfg = LaneFitConfig(
        min_lane_pixels=120,
        window_margin_px=100,
        meters_per_pixel_x=0.05,
        meters_per_pixel_y=0.05,
    )

    scenarios = [
        ("车辆位于车道正中", dict(
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=550, right_x_top=450, noise_std=0.0)),
        ("车辆偏右（车道整体左移 100 px）", dict(
            left_x_bottom=150, left_x_top=250,
            right_x_bottom=450, right_x_top=350, noise_std=0.0)),
        ("叠加高斯噪声 sigma=12", dict(
            left_x_bottom=250, left_x_top=350,
            right_x_bottom=550, right_x_top=450, noise_std=12.0, seed=7)),
    ]

    errors = []
    for i, (title, kwargs) in enumerate(scenarios, 1):
        err = run_scenario(title, i, cfg, outdir, kwargs)
        if err is not None:
            errors.append(err)

    print("=" * 74)
    print("汇总")
    print("=" * 74)
    if errors:
        arr = np.array(errors)
        print(f"  {len(errors)}/{len(scenarios)} 个场景检出两侧车道线")
        print(f"  车道中心测量误差    : 平均 {arr.mean():+.2f} px，"
              f"绝对值最大 {np.abs(arr).max():.2f} px")
        print(f"  折算成距离          : 最大 {np.abs(arr).max() * 0.05:.3f} m"
              f"（按 0.05 m/px）")
    else:
        print("  没有场景成功检出，请检查算法参数")
    print(f"  结果图片目录        : {outdir}")
    print()
    print("如果图片是 .ppm 格式（未装 Pillow），转成 PNG 的命令：")
    print("  sudo apt install -y imagemagick && convert demo_1.ppm demo_1.png")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
