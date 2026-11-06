"""测试收集辅助：把本包与同目录的 uav_mujoco_sim 加入 sys.path。

uav_mujoco_sim 在 colcon 工作空间里由安装目录提供；直接在源码树里跑
pytest 时它还没有被安装，导入会失败。这里只影响测试收集，不改变运行逻辑。
"""

import os
import sys

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIBLING_ROOT = os.path.abspath(os.path.join(PACKAGE_ROOT, "..", "uav_mujoco_sim"))

for path in (PACKAGE_ROOT, SIBLING_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)
