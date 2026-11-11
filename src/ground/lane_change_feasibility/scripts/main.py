#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lane_change_feasibility 的入口（仓库约定：launch 从这里进）。"""

from __future__ import division, print_function

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from lane_change_node import main


if __name__ == "__main__":
    sys.exit(main())
