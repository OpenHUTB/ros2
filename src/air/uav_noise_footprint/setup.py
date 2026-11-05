#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_noise_footprint"

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
    install_requires=["setuptools", "numpy", "mujoco", "matplotlib"],
    zip_safe=True,
    maintainer="黄家欣",
    maintainer_email="326516287+usernamekkyy@users.noreply.github.com",
    description="城市低空噪声足迹评估与航线降噪（MuJoCo 闭环验证）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_noise_footprint.cli:main",
            "noise_monitor = uav_noise_footprint.ros.noise_node:main",
        ],
    },
)
