#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""课程约定的入口（包根 `main.py`）：只做转发，实现放在 `scripts/cli.py`。

为什么要这一层：
  * 作业要求入口以 `main.` 开头且放在模块目录下，`roslaunch` / `rosrun` / 直接
    `python3 main.py` 三种方式行为一致；
  * 原来的实现把逻辑写在 `scripts/main.py` 里并且只支持 GUI（`mujoco.viewer`），
    在没有图形界面的机器上根本跑不起来。现在默认走离线批跑，`--gui` 才开窗口。

用法：
    python3 main.py                      # 离线批跑 + 导出 CSV/PNG
    python3 main.py --gui                # 可视化窗口
    python3 main.py --selftest           # 单元测试
    python3 main.py --v-ego 12 --ttc 6   # 改工况
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_HERE, "scripts")
for _p in (_HERE, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
