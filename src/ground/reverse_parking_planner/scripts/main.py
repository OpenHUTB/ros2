#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""包入口：把脚本目录加进 sys.path 后交给节点模块。

仓库约定 ``main.`` 开头的文件作为 launch 的入口，实际逻辑放在
parking_planner_node.py，核心几何在 parking_core.py。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
