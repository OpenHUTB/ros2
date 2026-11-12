# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "mujoco_hopper_slip_control"

# 版本号只维护一处：从包内 config.py 的 VERSION 读取。
# 注意不要改回解析 __init__.py —— 那里的 __version__ = VERSION 是变量引用而非字符串字面量，
# 正则匹配不到会让 setup.py 抛 AttributeError，而 colcon 会把真实错误吞掉、只报
# 「exited with code 1」。test/test_standalone.py 有一条断言守着这个假设。
_config_text = Path(__file__).resolve().parent.joinpath(
    package_name, "config.py").read_text(encoding="utf-8")
_match = re.search(r'^VERSION\s*=\s*"([^"]+)"', _config_text, re.MULTILINE)
if _match is None:
    raise RuntimeError(
        f"无法从 {package_name}/config.py 解析出 VERSION，"
        '请确认文件中有形如 VERSION = "1.0.0" 的一行')
_version = _match.group(1)

setup(
    name=package_name,
    version=_version,
    packages=[package_name],
    data_files=[
        # ament 索引，缺少它 colcon 不会生成 ament_prefix_path 钩子，
        # source 之后 ros2 run / ros2 launch 会报 Package not found
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 场景 XML 随包安装，sim.find_model() 会回退到 share 下查找
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jerrytsts",
    maintainer_email="jerrytsts@users.noreply.github.com",
    description="MuJoCo 单足弹跳机器人：SLIP 模型 + Raibert 落脚点 + 能量伺服，ROS 2 封装",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hopper_slip_node = mujoco_hopper_slip_control.ros_node:main",
            "hopper_viewer = mujoco_hopper_slip_control.viewer:main",
        ],
    },
)
