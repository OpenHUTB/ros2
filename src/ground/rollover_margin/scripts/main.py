#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""入口：把脚本目录加进 sys.path 后转交给 rollover_node.main。"""
from __future__ import division, print_function

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rollover_node import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())
