#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置。

本模块不依赖 ROS：`ros2 run` / `ros2 launch` 只是"多一种启动方式"。
离线批跑只需要 numpy；pygame 与 matplotlib 都是可选依赖。
"""

from glob import glob

from setuptools import setup

package_name = "car_2d_target_tracker"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="OpenHUTB",
    maintainer_email="openhutb@example.com",
    description="2D 小车目标追踪：运动学仿真与离线指标评测",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "car_2d_target_tracker = car_2d_target_tracker.cli:main",
        ],
    },
)
