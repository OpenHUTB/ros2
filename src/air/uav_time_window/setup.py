#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_time_window"

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
    maintainer="黄家欣",
    maintainer_email="326516287+usernamekkyy@users.noreply.github.com",
    description="多机任务 4D 时间窗分配与延误均衡（MuJoCo 闭环验证）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_time_window.cli:main",
            "schedule = uav_time_window.ros.schedule_node:main",
        ],
    },
)
