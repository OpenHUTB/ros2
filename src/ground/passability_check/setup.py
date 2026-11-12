# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "passability_check"

# 版本号只维护一处：从包内 __init__.py 读取
_init_text = Path(package_name, "__init__.py").read_text(encoding="utf-8")
_version = re.search(r'__version__\s*=\s*"([^"]+)"', _init_text).group(1)

setup(
    name=package_name,
    version=_version,
    packages=[package_name],
    data_files=[
        # ament 索引，缺少它 ros2 run / ros2 launch 找不到本功能包
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config", "scenarios"),
         glob("config/scenarios/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiaohua745-blip",
    maintainer_email="jiaohua745-blip@users.noreply.github.com",
    description="车辆几何通过性核算：接近角 / 离去角 / 纵向通过角 / 托底，纯标准库实现",
    license="MIT",
    entry_points={
        "console_scripts": [
            "passability_check_node = passability_check.node:main",
        ],
    },
)
