#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""turning_sweep_check 的可执行入口：roslaunch 的 type 指向本文件。"""

from __future__ import absolute_import

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from turning_node import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
