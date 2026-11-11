#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_inspection_cover"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tools"]),
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
    description="Transmission-corridor UAV inspection coverage planning "
                "(MuJoCo closed-loop verification)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_inspection_cover.cli:main",
            "inspect = uav_inspection_cover.ros.inspect_node:main",
        ],
    },
)
