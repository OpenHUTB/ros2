# -*- coding: utf-8 -*-
"""conftest.py —— 让 test/ 下的用例能按包导入 `reacher_track.*`。

包源码根目录是 `src/reacher_track/`，包本体是 `src/reacher_track/reacher_track/`。
pytest 默认只把 `test/` 加进 sys.path，因此这里显式补上包根目录，
这样 `python3 -m pytest test/ -q` 与 CI 里的调用方式一致，不依赖 CWD。
"""
import os
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)
