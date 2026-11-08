#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""源码直跑入口：python3 main.py run|demo|selftest，与 ros2 run 等价。"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from uav_link_aware_degradation.core.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
