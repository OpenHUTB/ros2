#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_speed_control 包根入口。

仓库约定每个模块在包目录下提供以 main 开头的入口，因此这里放一个薄封装，
实际逻辑仍在 scripts/main.py 中，避免两份实现走偏。

用法::

    python3 main.py --mode node     # 启动纵向速度控制节点
    python3 main.py --mode demo     # 离线演示，不需要 ROS / CARLA
    python3 main.py --mode check    # 参数与限速表自检
"""

from __future__ import annotations

import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "scripts", "main.py")


def main():
    if not os.path.isfile(TARGET):
        sys.stderr.write("[错误] 找不到 %s\n" % TARGET)
        return 1

    sys.path.insert(0, os.path.join(HERE, "scripts"))
    sys.argv[0] = TARGET
    runpy.run_path(TARGET, run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main())
