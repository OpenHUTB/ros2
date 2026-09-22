#!/usr/bin/env python3
"""CARLA 0.9.16 无人车四次作业 · ament Python 打包。

使本仓库可作为 ROS2 包被 colcon build，随后可用
`ros2 launch carla_assignment <launch 文件>` 以标准 ROS2 方式逐个启动四个作业。

launch 文件（Python）位于各作业目录的 launch/ 下，通过包数据收集进 install。
"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "carla_assignment"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 收集四个作业的 ROS2 launch 文件
        ("share/" + package_name + "/launch",
         glob("01_control/launch/*.launch.py")
         + glob("02_perception/launch/*.launch.py")
         + glob("03_navigation/launch/*.launch.py")
         + glob("04_end_to_end/launch/*.launch.py")
         + glob("05_reports/launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="CARLA 0.9.16 unmanned-vehicle NN assignments (4 tasks)",
    license="MIT",
    entry_points={
        "console_scripts": [
            # 让每个作业也能以 ros2 run 方式直接启动
            "carla_control = carla_assignment.01_control.main:main",
            "carla_perception = carla_assignment.02_perception.main:main",
            "carla_navigation = carla_assignment.03_navigation.main:main",
            "carla_end_to_end = carla_assignment.04_end_to_end.main:cli",
        ],
    },
)
