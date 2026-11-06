#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``python -m uav_mujoco_sim.sim`` 入口。

不走 ``-m ...sim.server``：runpy 会报 "found in sys.modules after import of package" 告警。
"""
from .server import main

if __name__ == "__main__":
    raise SystemExit(main())
