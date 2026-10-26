#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定板摆位次数扫描：回答"需要摆几次板才能标得准"。

用法::

    python3 -m lidar_camera_calibration.sweep_views --trials 200
    python3 -m lidar_camera_calibration.sweep_views --views 1,2,3,4,6
"""

from __future__ import annotations

import argparse
import csv
import sys

from .calibration_core import views_sweep


def _parse_views(text: str):
    return [int(x) for x in text.replace('，', ',').split(',') if x.strip()]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description='标定板摆位次数扫描')
    p.add_argument('--trials', type=int, default=200)
    p.add_argument('--views', default='1,2,3,4,6',
                   help='要扫描的摆位次数，逗号分隔')
    p.add_argument('--csv', default=None)
    ns = p.parse_args(sys.argv[1:] if argv is None else argv)

    rows = views_sweep(trials=ns.trials, views_list=_parse_views(ns.views))

    print('## 摆位次数 vs 精度（%d 次蒙特卡洛，中位数，1 px + 1 cm）\n'
          % ns.trials)
    print('| 摆位次数 | 总点数 | 旋转误差(°) | 平移误差(m) | 重投影 RMSE(px) |')
    print('|---|---|---|---|---|')
    for r in rows:
        print('| %d | %d | %.3f | %.4f | %.3f |'
              % (r['views'], r['n_points'], r['rot_error_deg'],
                 r['trans_error_m'], r['rmse_px']))

    if ns.csv:
        with open(ns.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['views', 'n_points', 'rot_error_deg',
                        'trans_error_m', 'rmse_px', 'trials'])
            for r in rows:
                w.writerow([r['views'], r['n_points'], r['rot_error_deg'],
                            r['trans_error_m'], r['rmse_px'], r['trials']])
        print('\n结果已写入: %s' % ns.csv)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
