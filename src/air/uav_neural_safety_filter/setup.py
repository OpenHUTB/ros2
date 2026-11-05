#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import setup, find_packages

package_name = "uav_neural_safety_filter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "docs"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name),
         ["package.xml", "README.md", "main.py"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "data"), glob("data/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eeexty",
    maintainer_email="eeexty@users.noreply.github.com",
    description="神经安全过滤器：激光距离栅格 + 行为克隆的安全层（ROS2 Humble）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_neural_safety_filter.cli:main",
            "safety_node = uav_neural_safety_filter.ros.safety_node:main",
            "nominal_node = uav_neural_safety_filter.ros.nominal_node:main",
        ],
    },
)
