#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_dynamic_safety_guard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name),
         ["package.xml", "README.md", "main.py"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "data"), glob("data/*.npz")),
        (os.path.join("share", package_name, "data"), glob("data/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yanfan117",
    maintainer_email="3037013261@qq.com",
    description="动态障碍下的时序神经安全层：三帧激光栅格 + 行为克隆（ROS2 Humble）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_dynamic_safety_guard.cli:main",
            "guard_node = uav_dynamic_safety_guard.ros.guard_node:main",
            "nominal_node = uav_dynamic_safety_guard.ros.nominal_node:main",
        ],
    },
)
