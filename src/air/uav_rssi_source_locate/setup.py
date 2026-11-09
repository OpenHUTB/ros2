#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_rssi_source_locate"

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
    description="UAV search and RSSI trilateration of a hidden radio beacon "
                "with MuJoCo closed loop. 隐藏无线信标的无人机搜索与 RSSI "
                "三边定位：扩张方形搜索、测量圈收缩、闭环验证。",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_rssi_source_locate.cli:main",
            "scan_node = uav_rssi_source_locate.ros.scan_node:main",
        ],
    },
)
