#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""噪声敏感性扫描：像素噪声 × 测距噪声 -> 外参误差。

用法::

    python3 -m lidar_camera_calibration.sweep_noise --trials 200
    python3 -m lidar_camera_calibration.sweep_noise --trials 50 --csv /tmp/noise.csv
"""

from __future__ import annotations

import argparse
import csv
import sys

from .calibration_core import noise_sweep


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='外参标定噪声敏感性扫描')
    p.add_argument('--trials', type=int, default=200,
                   help='每个噪声组合的蒙特卡洛次数')
    p.add_argument('--views', type=int, default=3)
    p.add_argument('--csv', default=None, help='结果 CSV 路径')
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    rows = noise_sweep(trials=ns.trials, views=ns.views)

    print('## 噪声敏感性（%d 次蒙特卡洛，中位数，%d 个摆位）\n'
          % (ns.trials, ns.views))
    print('| 像素噪声(px) | 测距噪声(m) | 旋转误差(°) | 平移误差(m) | 重投影 RMSE(px) |')
    print('|---|---|---|---|---|')
    for r in rows:
        print('| %.1f | %.2f | %.3f | %.4f | %.3f |'
              % (r['pixel_noise'], r['range_noise'], r['rot_error_deg'],
                 r['trans_error_m'], r['rmse_px']))

    if ns.csv:
        with open(ns.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['pixel_noise', 'range_noise', 'angle_noise_deg',
                        'rot_error_deg', 'trans_error_m', 'rmse_px', 'trials'])
            for r in rows:
                w.writerow([r['pixel_noise'], r['range_noise'],
                            r['angle_noise_deg'], r['rot_error_deg'],
                            r['trans_error_m'], r['rmse_px'], r['trials']])
        print('\n结果已写入: %s' % ns.csv)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
