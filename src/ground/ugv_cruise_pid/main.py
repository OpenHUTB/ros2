#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ugv_cruise_pid 包根入口。

仓库约定每个模块在包目录下提供以 main 开头的入口，这里放一个薄封装，
实际逻辑在 scripts/cruise_pid_node.py 中，避免两份实现走偏。

用法::

    python3 main.py
"""

from __future__ import annotations

import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "scripts", "cruise_pid_node.py")


def main():
    if not os.path.isfile(TARGET):
        sys.stderr.write("[错误] 找不到 %s\n" % TARGET)
        return 1

    scripts_dir = os.path.join(HERE, "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)

    runpy.run_path(TARGET, run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
