#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""launch 入口：满足仓库「入口以 main. 开头」的约定。"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from scenario_runner_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
