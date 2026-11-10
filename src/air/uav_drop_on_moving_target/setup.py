#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_drop_on_moving_target"

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
    description="Release timing for delivering supplies onto a moving "
                "convoy: analytic ballistics with wind drift, future-position "
                "prediction and MuJoCo closed-loop verification."
                " 给行驶中的车队投送物资：平抛弹道 + 车辆未来位置预测 + "
                "投放时刻解算，MuJoCo 闭环复核落点。",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_drop_on_moving_target.cli:main",
            "drop_node = uav_drop_on_moving_target.ros.drop_node:main",
        ],
    },
)
