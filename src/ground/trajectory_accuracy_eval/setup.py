#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""catkin（ROS 1）Python 打包配置。

配合 CMakeLists.txt 的 ``catkin_python_setup()``，把 ``traj_eval`` 包装到
``devel/lib/python3/dist-packages``，这样 ``roslaunch`` 启动的节点脚本才能
``import traj_eval``。离线使用（``python3 main.py``）不依赖该文件。
"""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # traj_eval 直接位于包目录下（不在 src/ 子目录里），
    # 因此不需要 package_dir；catkin 会把它装到 devel/lib/python3/dist-packages。
    packages=["traj_eval"],
)

setup(**d)
