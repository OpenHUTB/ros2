# -*- coding: utf-8 -*-
"""pytest 配置：把包根加入 sys.path，使 `pytest test/` 与
`python test/test_bezier.py` 两种跑法都能 import 到 bezier_trajectory。"""

import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)