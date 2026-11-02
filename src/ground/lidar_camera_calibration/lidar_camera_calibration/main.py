#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一入口：跑一遍标准标定并打印摘要（不依赖 ROS）。

等价于::

    python3 -m lidar_camera_calibration.run_calibration --views 3
"""

from __future__ import annotations

import sys

from .run_calibration import main as run_main


def main(argv=None) -> int:
    print('相机-激光雷达外参标定：标准演示（3 个摆位，1 px + 1 cm 噪声）\n')
    args = sys.argv[1:] if argv is None else argv
    if not args:
        args = ['--views', '3', '--pixel-noise', '1.0', '--range-noise', '0.01']
    return run_main(args)


if __name__ == '__main__':
    raise SystemExit(main())
