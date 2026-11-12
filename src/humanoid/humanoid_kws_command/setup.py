# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "humanoid_kws_command"

# 版本号只维护一处：从 config.py 的 VERSION 读取（不要改成解析 __init__.py，
# 那里是 __version__ = VERSION 变量引用，正则匹配不到会让 colcon 只报一句
# 「exited with code 1」）。
_cfg = Path(__file__).resolve().parent.joinpath(package_name, "config.py").read_text(encoding="utf-8")
_m = re.search(r'^VERSION\s*=\s*"([^"]+)"', _cfg, re.MULTILINE)
if _m is None:
    raise RuntimeError("无法从 config.py 解析出 VERSION")

setup(
    name=package_name,
    version=_m.group(1),
    packages=[package_name],
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "data"), glob("data/*.npz")),
        (os.path.join("share", package_name, "weights"), glob("weights/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jerrytsts",
    maintainer_email="jerrytsts@users.noreply.github.com",
    description="离线语音指令控制：log-mel + 小型 MLP 的关键词识别（KWS）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["kws_command_node = humanoid_kws_command.ros_node:main"]},
)