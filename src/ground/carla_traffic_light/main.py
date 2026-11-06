#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_traffic_light 包根入口。

仓库约定每个模块在包目录下提供以 main 开头的入口，因此这里放一个薄封装，
实际逻辑仍在 scripts/main.py 中，避免两份实现走偏。

用法::

    python3 main.py                  # 离线仿真，不需要 CARLA
    python3 main.py --all-lights     # 红/黄/绿三色依次跑
    python3 main.py --carla          # 连 CARLA 真车
    roslaunch carla_traffic_light main.launch
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
