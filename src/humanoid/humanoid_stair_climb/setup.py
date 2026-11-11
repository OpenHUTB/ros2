# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "humanoid_stair_climb"

# 版本号只维护一处：从包内 __init__.py 读取，避免 setup.py 与 --version 输出不一致
_init_text = Path(__file__).resolve().parent.joinpath(package_name, "__init__.py").read_text(
    encoding="utf-8")
_version = re.search(r'__version__\s*=\s*"([^"]+)"', _init_text).group(1)

setup(
    name=package_name,
    version=_version,
    packages=[package_name],
    data_files=[
        # ament 索引，缺少它 colcon 不会生成 ament_prefix_path 钩子，
        # source 之后 ros2 run / ros2 launch 会报 Package not found
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 场景 XML 随包安装，sim_core.default_model_path() 会回退到 share 下查找
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="L2008-by",
    maintainer_email="L2008-by@users.noreply.github.com",
    description="人形上下台阶步态仿真：解析 IK + 分段足端轨迹，MuJoCo 六级台阶场景",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stair_climb_node = humanoid_stair_climb.ros_node:main",
        ],
    },
)
