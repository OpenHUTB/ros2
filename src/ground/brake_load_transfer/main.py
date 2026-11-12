#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""脚本入口：``python3 main.py ...``

等价于 ``python3 -m brake_load_transfer.main``，
单独放一个文件是为了让不熟悉 `-m` 的同学也能直接双击/命令行运行。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from brake_load_transfer.main import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
