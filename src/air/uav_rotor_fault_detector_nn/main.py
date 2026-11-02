#!/usr/bin/env python3
"""模块入口：把 ``src/air/uav_rotor_fault_detector_nn`` 放进 ``sys.path`` 后转给 CLI。

两种部署方式都要能用：

* 源码树里直接 ``python main.py --selftest``；
* ``colcon build`` 之后 ``ros2 run`` / ``ros2 launch``，此时包被装进
  ``<ws>/install/lib/python3.x/site-packages``，本文件的相对位置也变了。

所以这里不写死路径，而是按"哪个候选目录下面真的有一个同名包目录"来选。
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
    if os.path.isdir(os.path.join(_root, "uav_rotor_fault_detector_nn")):
        sys.path.insert(0, _root)
        break

from uav_rotor_fault_detector_nn.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
