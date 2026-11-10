#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_actuator_fault_sim 打包脚本（ament_python，ROS2 Humble）。"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_actuator_fault_sim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test", "tools", "docs"]),
    data_files=[
        # ament 资源索引：ros2 run / ros2 launch 据此找到本包
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        # 入口与元信息装入 share，安装布局下 launch 靠 main.py 启动
        (os.path.join("share", package_name),
         ["package.xml", "README.md", "main.py"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        # 训练/评估产出的轻量权重与指标表（没有时为空列表，不影响安装）
        (os.path.join("share", package_name, "results"),
         glob("results/*.csv")),
    ],
    install_requires=["setuptools", "numpy>=1.21", "mujoco>=3.0.0"],
    zip_safe=True,
    maintainer="Fortranquility",
    maintainer_email="2920111320@qq.com",
    description="无人机执行器/传感器故障注入仿真层（ROS2 Humble + MuJoCo）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main = uav_actuator_fault_sim.cli:main",
            # ROS 节点单独一个入口，launch 里用它
            "fault_node = uav_actuator_fault_sim.ros_node:main",
        ],
    },
)
