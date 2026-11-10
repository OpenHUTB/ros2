#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""便捷入口：等价于 ``python3 scripts/main.py``。

真正的实现和 roslaunch 用的入口都在 ``scripts/main.py``——ROS 1 的 roslaunch
只会去 ``<包>/scripts/`` 下找 ``type="main.py"``，所以入口必须在 scripts 里。
本文件只是让在模块根目录下直接 ``python3 main.py`` 也能跑。
"""

from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "scripts"))

from main import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
