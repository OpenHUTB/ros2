#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_moving_target_follow"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tools", "docs"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name),
         ["package.xml", "README.md", "main.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "mujoco"],
    zip_safe=True,
    maintainer="蒋宇鑫",
    maintainer_email="327592576+KKK555-MAX@users.noreply.github.com",
    description="UAV standoff following of a moving ground target with "
                "analytic reference generation and MuJoCo closed loop."
                " 移动地面目标的后方等距跟随：截获预测、standoff 参考生成、"
                "MuJoCo 闭环验证。",
    license="MIT",
    entry_points={
        "console_scripts": [
            "main = uav_moving_target_follow.cli:main",
            "follow_node = uav_moving_target_follow.ros.follow_node:main",
        ],
    },
)
