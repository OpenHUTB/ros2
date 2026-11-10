#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_auto_light 统一入口（仓库约定：入口为 main.*）。

演示 CARLA 地面载具的**灯光自动控制**：自动大灯（黄昏/夜间/降水时开启）、雾灯、
转向灯（打方向点亮、回正自动熄灭）、刹车灯。

用法::

    # 完整演示：白天 → 黄昏 → 夜间 → 暴雨 四个阶段，每个阶段 6 秒
    python3 main.py --host 192.168.148.1

    # 每个阶段 4 秒（更快）
    python3 main.py --host 192.168.148.1 --phase-seconds 4

    # 换车型、换地图
    python3 main.py --host 192.168.148.1 --vehicle vehicle.bmw.grandtourer --town Town05
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_auto_light.app import main      # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
