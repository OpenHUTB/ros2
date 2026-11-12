"""pytest 配置：把 scripts/ 加入 sys.path，使 test/ 下可直接 import 项目模块。

两种运行方式都成立：
    cd src/chap8/mujoco_2link_impedance_control && python3 -m pytest test -q
    cd src/chap8/mujoco_2link_impedance_control && python3 -m unittest discover -s test
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(os.path.dirname(HERE), "scripts")
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
