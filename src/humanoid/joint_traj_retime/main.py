#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""脚本入口：``python3 main.py``（等价于 ``python3 -m joint_traj_retime.main``）。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from joint_traj_retime.main import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
