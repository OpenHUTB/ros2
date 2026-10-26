#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""投影叠加验证：把点云按"估计外参"投影到合成相机图像上，输出 PNG。

真值投影画绿圈、估计外参投影画红点；两者重合说明外参标定正确。
同时生成两张误差曲线图（噪声扫描、摆位扫描）。

用法::

    python3 -m lidar_camera_calibration.project_cloud --out-dir /tmp/calib_out
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

from .calibration_core import (
    DEFAULT_K,
    DEFAULT_T,
    build_scene,
    calibrate,
    default_R,
    overlay_projection,
    project,
    render_synthetic_image,
    simulate_lidar_noise,
)


def _plot_noise(rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    pixel_levels = sorted({r['pixel_noise'] for r in rows})
    fig, ax = plt.subplots(figsize=(7, 5))
    for r_noise in sorted({r['range_noise'] for r in rows}):
        xs, ys = [], []
        for px in pixel_levels:
            hit = [r for r in rows
                   if r['pixel_noise'] == px and r['range_noise'] == r_noise]
            if hit:
                xs.append(px)
                ys.append(hit[0]['rot_error_deg'])
        ax.plot(xs, ys, marker='o', label='range noise %.0f cm' % (r_noise * 100))
    ax.set_xlabel('pixel noise (px)')
    ax.set_ylabel('rotation error (deg)')
    ax.set_title('Extrinsic calibration: rotation error vs noise')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_views(rows, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 5))
    xs = [r['views'] for r in rows]
    ax.plot(xs, [r['rot_error_deg'] for r in rows], marker='o',
            label='rotation error (deg)')
    ax.plot(xs, [r['trans_error_m'] * 100 for r in rows], marker='s',
            label='translation error (cm)')
    ax.set_xlabel('number of board views')
    ax.set_ylabel('error')
    ax.set_title('How many board views are needed?')
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='外参标定投影叠加与误差曲线')
    p.add_argument('--out-dir', default='/tmp/lidar_camera_calibration')
    p.add_argument('--views', type=int, default=3)
    p.add_argument('--pixel-noise', type=float, default=1.0)
    p.add_argument('--range-noise', type=float, default=0.01)
    p.add_argument('--seed', type=int, default=0)
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    os.makedirs(ns.out_dir, exist_ok=True)

    # 1) 用标定结果把点云投影到合成图像
    points, poses = build_scene(ns.views)
    res = calibrate(ns.views, ns.pixel_noise, ns.range_noise,
                    0.1, ns.seed, DEFAULT_K)
    image = render_synthetic_image(points, poses, default_R(), DEFAULT_T)
    overlaid = overlay_projection(image, points, res.R_est, res.t_est)
    overlay_path = os.path.join(ns.out_dir, 'projection_overlay.png')
    cv2.imwrite(overlay_path, overlaid)

    print('标定结果: 旋转误差 %.3f°，平移误差 %.4f m，重投影 RMSE %.3f px'
          % (res.rot_error_deg, res.trans_error_m, res.rmse_px))
    print('投影叠加图: %s' % overlay_path)

    # 2) 噪声曲线
    from .calibration_core import noise_sweep, views_sweep
    noise_rows = noise_sweep(trials=60, views=ns.views)
    noise_path = os.path.join(ns.out_dir, 'noise_sweep.png')
    _plot_noise(noise_rows, noise_path)
    print('噪声曲线: %s' % noise_path)

    # 3) 摆位次数曲线
    view_rows = views_sweep(trials=60)
    views_path = os.path.join(ns.out_dir, 'views_sweep.png')
    _plot_views(view_rows, views_path)
    print('摆位曲线: %s' % views_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
