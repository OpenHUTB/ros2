#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""倒车雷达分级告警包的启动入口。

按仓库约定，每个模块的入口都要以 main. 开头（main.py / main.cpp / main.sh），
roslaunch 里 type 写的就是这个文件名。
"""

from __future__ import division, print_function, unicode_literals

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from alert_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
