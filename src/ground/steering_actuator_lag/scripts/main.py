#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""steering_actuator_lag 的入口（仓库约定：run 时从这里进）。"""

from __future__ import division, print_function

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from actuator_node import main


if __name__ == "__main__":
    sys.exit(main())
