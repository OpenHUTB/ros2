#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""课程约定的入口（包根 `main.py`）：只做转发，实现放在 `car_2d_target_tracker/cli.py`。

原来的 `main.py` 是这个模块里唯一的文件，且**必须在 pygame 窗口里跑**：
没有 pygame、没有显示器的机器上直接 ImportError / 无法开窗。现在默认走离线
批跑（只用 numpy），`--gui` 才需要 pygame。

    python3 main.py                      # 离线批跑：三场景 × 两个控制器，导出 CSV/PNG
    python3 main.py --gui                # pygame 窗口（交互）
    python3 main.py --selftest           # 单元测试
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from car_2d_target_tracker.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
