#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""catkin（ROS 1）Python 打包配置。

配合 CMakeLists.txt 中的 ``catkin_python_setup()``，把
``lateral_control_benchmark`` 这个 Python 包安装到 ``devel/lib/python3/dist-packages``，
这样 ``roslaunch`` 启动的节点脚本才能 ``import lateral_control_benchmark``。

同时提供 ``lateral_control_node`` 控制台入口，可以脱离 roslaunch 直接运行。
"""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["lateral_control_benchmark"],
    package_dir={"": ""},
    scripts=[],
)

setup(**d)
