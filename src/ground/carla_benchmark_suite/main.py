#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""作业五 · CARLA 综合基准评测套件 —— 课程约定的 main.* 入口。

实现全部在 Python 包 `carla_benchmark_suite/main.py` 里，本文件只是一层薄壳：

  * 课程要求入口叫 `main.*`，所以包根必须有一个 main.py；
  * 但 ROS 2 节点是按 `carla_benchmark_suite.main` 导入的，只有放进 Python 包内
    （`packages=[package_name]` 才会把它装进 site-packages）才能在 `ros2 launch`
    之后被找到。放在包根的话 `setup.py` 不会安装它，节点启动后加载评测模块会
    失败，指标全空。

用法与原来的 main.py 完全一致，例如：
    python3 main.py --list
    python3 main.py --benchmark --out report.json
    python3 main.py --module navigation -- --demo --headless
"""

import os
import sys

# 直接 `python3 main.py` 时脚本所在目录就在 sys.path[0]，这里再显式加一次，
# 保证被 roslaunch 按 shebang 执行、或从别的目录调用时也能找到包。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from carla_benchmark_suite.main import *          # noqa: F401,F403,E402
from carla_benchmark_suite.main import main        # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
