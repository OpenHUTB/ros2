#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""roslaunch 的入口：把包内目录加进 sys.path 后启动节点。"""

from __future__ import absolute_import

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from speed_node import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())
