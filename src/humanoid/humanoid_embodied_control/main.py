#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""具身人形 —— 课程约定主入口（main.py）。

本文件只是转发器：真正实现位于 ``scripts/main.py``。之所以保留这一层，
是因为仓库要求"每个模块必须支持 launch 启动，入口为 main 开头，如
main.py / main.sh / main.bat"，而 ROS launch 又需要按可执行脚本调用，
两者用同一份参数解析器时放在 scripts/ 下更清晰。

用法与 ``scripts/main.py`` 完全一致，例如::

    python3 main.py --mode sim --seconds 30
    python3 main.py --mode offline
    python3 main.py --help
"""

from __future__ import annotations

import os
import runpy
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_ROOT, "scripts")
for path in (_ROOT, _SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

if __name__ == "__main__":
    sys.argv[0] = os.path.join(_SCRIPTS, "main.py")
    runpy.run_path(sys.argv[0], run_name="__main__")
