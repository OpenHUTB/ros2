"""pytest 配置：把 scripts/ 加入 sys.path，使 test/ 下可直接 import 项目模块。

这样两种运行方式都成立：
    cd src/chap8/ros_mujoco_arm_pickplace && python3 -m pytest test -q
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(os.path.dirname(HERE), "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
