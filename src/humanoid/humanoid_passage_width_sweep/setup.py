# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "humanoid_passage_width_sweep"

# 版本号只维护一处：从 config.py 的 VERSION 读取（不要改去解析 __init__.py，
# 那里的 __version__ = VERSION 是变量引用，正则匹配不到会让 colcon 只报
# 一句无头无尾的「exited with code 1」）。
_config = Path(__file__).resolve().parent.joinpath(package_name, "config.py").read_text(
    encoding="utf-8")
_m = re.search(r'^VERSION\s*=\s*"([^"]+)"', _config, re.MULTILINE)
if _m is None:
    raise RuntimeError("无法从 config.py 解析出 VERSION")
_version = _m.group(1)

setup(
    name=package_name,
    version=_version,
    packages=[package_name],
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jerrytsts",
    maintainer_email="jerrytsts@users.noreply.github.com",
    description="人形足迹下的局部避障算法对比与窄通道宽度扫描（MuJoCo + ROS 2）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "passage_sweep_node = humanoid_passage_width_sweep.ros_node:main",
            "passage_viewer = humanoid_passage_width_sweep.viewer:main",
        ],
    },
)