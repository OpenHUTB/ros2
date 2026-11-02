#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_gate_center_servo"

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
        (os.path.join("share", package_name, "data"), glob("data/*.json")),
    ],
    install_requires=["setuptools", "numpy", "matplotlib"],
    zip_safe=True,
    maintainer="eeexty",
    maintainer_email="eeexty@users.noreply.github.com",
    description="单门深度视觉伺服：门中心回归 + 比例控制",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_gate_center_servo.cli:main",
            "gate_servo_node = uav_gate_center_servo.ros.gate_servo_node:main",
        ],
    },
)
