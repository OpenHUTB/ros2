#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_emergency_drop"

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
    description="UAV emergency supply drop point selection and release "
                "timing with analytic ballistics and MuJoCo closed loop."
                " 应急物资投送点选择与投放时机决策：平抛弹道解析解、"
                "贪心+交换选点、投放区间判定与 MuJoCo 闭环验证。",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_emergency_drop.cli:main",
            "drop_node = uav_emergency_drop.ros.drop_node:main",
        ],
    },
)
