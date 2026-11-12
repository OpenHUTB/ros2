#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""课程约定的入口（包根 `main.py`，只做转发）。

实现放在 Python 包里（`mujoco_balance_car/cli.py`），原因：`setup.py` 用的是
`packages=[package_name]`，`colcon build` 只会把这个目录装进 site-packages，
包根的文件不会被安装；而 `ros2 run`/`ros2 launch` 是按包内模块导入的。若实现
留在包根，安装后就找不到入口。

用法与包内实现完全一致，例如：

    python3 main.py                      # 离线批跑 + 导出 CSV/PNG
    python3 main.py --gui                # 可视化窗口
    python3 main.py --selftest           # 单元测试
    python3 main.py --lean 0.2 --duration 8 --save_dir shots
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from mujoco_balance_car.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
