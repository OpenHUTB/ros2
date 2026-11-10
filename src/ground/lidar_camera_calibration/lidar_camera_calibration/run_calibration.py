#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单次外参标定：生成合成场景、加噪声、PnP 求解、与真值对比。

用法::

    python3 -m lidar_camera_calibration.run_calibration
    python3 -m lidar_camera_calibration.run_calibration --views 3 \\
        --pixel-noise 1.0 --range-noise 0.01 --seed 0

也可以直接运行脚本（不需要 colcon 构建）::

    python3 run_calibration.py --views 3
"""

from __future__ import annotations

import argparse
import json
import sys

from .calibration_core import calibrate


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='相机-激光雷达外参标定（合成数据）')
    p.add_argument('--views', type=int, default=3,
                   help='标定板摆放次数（1..6，建议 >=3）')
    p.add_argument('--pixel-noise', type=float, default=1.0,
                   help='像素坐标噪声标准差（px）')
    p.add_argument('--range-noise', type=float, default=0.01,
                   help='激光测距噪声标准差（m）')
    p.add_argument('--angle-noise', type=float, default=0.1,
                   help='激光角噪声标准差（度）')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--json', default=None, help='把结果写入 JSON 文件')
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    res = calibrate(ns.views, ns.pixel_noise, ns.range_noise,
                    ns.angle_noise, ns.seed)
    print(res.to_text())

    if ns.json:
        payload = dict(
            views=res.views, n_points=res.n_points,
            pixel_noise=res.pixel_noise, range_noise=res.range_noise,
            angle_noise_deg=res.angle_noise_deg,
            rot_error_deg=res.rot_error_deg,
            trans_error_m=res.trans_error_m,
            rmse_px=res.rmse_px,
            R_est=res.R_est.tolist(), t_est=res.t_est.tolist(),
            R_true=res.R_true.tolist(), t_true=res.t_true.tolist(),
        )
        with open(ns.json, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        print('\n结果已写入: %s' % ns.json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
