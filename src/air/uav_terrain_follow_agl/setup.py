#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_terrain_follow_agl"

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
    install_requires=["setuptools", "numpy", "mujoco", "matplotlib", "pyyaml"],
    zip_safe=True,
    maintainer="蒋宇鑫",
    maintainer_email="327592576+KKK555-MAX@users.noreply.github.com",
    description="Terrain following at constant height above ground: radar "
                "altimeter latency compensation and DEM preview, with MuJoCo "
                "closed-loop verification. 低空地形跟随：恒定离地高度保持、"
                "雷达高度表延迟补偿与 DEM 前馈，MuJoCo 闭环校验。",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_terrain_follow_agl.cli:main",
            "terrain_follow_node = uav_terrain_follow_agl.ros.terrain_follow_node:main",
        ],
    },
)
