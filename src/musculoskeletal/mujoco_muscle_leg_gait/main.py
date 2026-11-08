#!/usr/bin/env python3
"""肌肉驱动下肢模块统一入口（仓库约定的 main.py）。

用法（在模块目录下）：
    python3 main.py check                 # 四个模型自检（单关节/双关节/单腿/双腿）
    python3 main.py run --scene l3        # 跑单个实验
    python3 main.py bench --out out       # 跑全部并导出指标 JSON
    python3 main.py gif --out gif         # 渲染动图

依赖：mujoco>=3.0, numpy；gif 另需 Pillow。
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from mujoco_muscle_leg_gait.cli import main  # noqa: E402

if __name__ == "__main__":
    main()
