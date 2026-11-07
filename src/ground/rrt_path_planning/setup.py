# -*- coding: utf-8 -*-
from setuptools import setup, find_packages

package_name = "rrt_path_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["pygame>=2.0"],
    entry_points={
        "console_scripts": [
            "main = " + package_name + ".main:main",
        ],
    },
)
