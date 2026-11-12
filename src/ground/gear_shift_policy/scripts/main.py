#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""入口脚本：保持与仓库「模块入口以 main. 开头」的约定。"""

from __future__ import print_function, unicode_literals

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gear_node import main as node_main


if __name__ == "__main__":
    node_main()
