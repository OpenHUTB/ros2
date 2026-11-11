#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ride_comfort_transmissibility 节点入口：参数、核算、打印、发布。"""

from __future__ import division, print_function, unicode_literals

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from ride_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
