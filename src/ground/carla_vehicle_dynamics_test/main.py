#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""carla_vehicle_dynamics_test 统一入口（仓库约定：入口为 main.*）。

在 CARLA 里对地面载具做三项**动力性测试**，支持多车型对比：

* ``accel`` —— 全油门 0→60 km/h 加速时间与平均加速度；
* ``brake`` —— 60→0 制动距离与平均减速度（与匀减速理论值对照）；
* ``turn``  —— 固定转向角的稳态转弯半径（与阿克曼理论值对照）；
* ``all``   —— 三项依次全部执行。

用法示例::

    # 单车型三项全测
    python3 main.py --mode all --host 192.168.148.1 --vehicle vehicle.tesla.model3

    # 多车型对比（自动循环，结果写入 CSV 与曲线图）
    python3 main.py --mode all --host 192.168.148.1 --vehicles all

    # 只测加速，目标 100 km/h
    python3 main.py --mode accel --host 192.168.148.1 --v-target-kmh 100
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_vehicle_dynamics_test.app import main      # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
