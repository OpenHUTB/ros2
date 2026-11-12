# -*- coding: utf-8 -*-
from glob import glob
from setuptools import setup, find_packages

package_name = "rrt_path_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        # 同一个 share 目录只写一次，避免安装空间里重复注册
        ("share/" + package_name,
            ["package.xml", "README.md", "requirements.txt",
             "main.py", "main.sh", "main.bat"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["pygame>=2.0"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = " + package_name + ".main:main",
        ],
    },
)