#!/usr/bin/env python3
"""源码树入口：把 ``src/air/uav_fault_e2e_thrust_policy`` 放进 ``sys.path`` 后转给 CLI。

和 ``setup.py`` 里 ``console_scripts`` 的 ``main = ...cli:main`` 是**两个入口、同一个实现**，
不是重复代码：

* 仓库约定每个模块根目录要有 ``main.py`` / ``main.sh`` / ``main.bat``，评审和同学
  直接 ``python main.py --selftest`` 就能跑，不需要先 ``colcon build``；
* 安装之后 ``ros2 run uav_fault_e2e_thrust_policy main`` 走的是 ``console_scripts``，
  由 setuptools 生成启动脚本，此时本文件不参与。

本文件只做路径引导，真正的命令行都在 ``uav_fault_e2e_thrust_policy/cli.py``。

两种部署方式都要能用，所以这里不写死路径，而是按"哪个候选目录下面真的有一个同名包目录"来选：
源码树里是模块目录本身，``colcon build`` 之后是 ``<ws>/install/lib/python3.x/site-packages``。
"""

import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_INSTALL_PREFIX = os.path.dirname(os.path.dirname(_HERE))
_ROOTS = [_HERE] + sorted(
    glob.glob(os.path.join(_INSTALL_PREFIX, "lib", "python*", "site-packages"))
)

for _root in _ROOTS:
    if os.path.isdir(os.path.join(_root, "uav_fault_e2e_thrust_policy")):
        sys.path.insert(0, _root)
        break

from uav_fault_e2e_thrust_policy.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
