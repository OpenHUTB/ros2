#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CLI 入口：与源码根目录 main.py 行为一致，便于 ``ros2 run ... main`` 调用。"""
from __future__ import annotations

import sys
from pathlib import Path

# colcon 安装后，本文件只需要 core 包；源码直跑时把仓库目录补进 sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from uav_link_aware_degradation.core.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
