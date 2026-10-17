#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import setup, find_packages

package_name = "uav_depth_gate_racing_e2e"

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
        # 预训练策略权重（离线复现用）
        (os.path.join("share", package_name, "data"), glob("data/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eeexty",
    maintainer_email="eeexty@users.noreply.github.com",
    description="深度图端到端穿越门竞速（DAgger + PPO，ROS2 Humble）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main = uav_depth_gate_racing_e2e.cli:main",
            "racing_node = uav_depth_gate_racing_e2e.ros.racing_node:main",
        ],
    },
)
