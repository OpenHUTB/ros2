#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""lateral_stability_margin 节点入口：读取参数、核算、打印、发布。"""

from __future__ import division, print_function, unicode_literals

import os
import sys

try:  # Python 2 相对导入兜底
    _HERE = os.path.dirname(os.path.abspath(__file__))
    if _HERE not in sys.path:
        sys.path.insert(0, _HERE)
    from stability_node import main
except ImportError:  # pragma: no cover
    from .stability_node import main

if __name__ == "__main__":
    sys.exit(main())
