#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""scene_object_layouter 的统一入口。

仓库约定入口文件名以 main. 开头，节点逻辑放在 layouter_node.py，
这里只负责把自身目录塞进 sys.path 再转发过去，方便 roslaunch 直接执行。
"""

from __future__ import print_function

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from layouter_node import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())
