#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""统一入口：把包内脚本目录加进 sys.path 后转交 trace_node.main。"""

from __future__ import absolute_import

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from trace_node import main


if __name__ == "__main__":
    sys.exit(main())
