#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import setup, find_packages

package_name = "uav_neural_vio_slam"

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
        # 预训练权重 + 示例数据（离线复现用）
        (os.path.join("share", package_name, "data"),
         glob("data/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eeexty",
    maintainer_email="eeexty@users.noreply.github.com",
    description="神经 VIO 前端 + 三后端 SLAM 对比（ROS2 Humble）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main = uav_neural_vio_slam.cli:main",
            "vio_node = uav_neural_vio_slam.ros.vio_node:main",
        ],
    },
)
