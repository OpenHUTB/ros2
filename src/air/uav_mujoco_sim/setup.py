#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import setup, find_packages

package_name = "uav_mujoco_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "docs"]),
    data_files=[
        # ament 资源索引，ros2 run / ros2 launch 据此找到本包
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        # main.py 装入 share：安装布局下 launch 靠它启动仿真服务端
        (os.path.join("share", package_name),
         ["package.xml", "README.md", "main.py"]),
        # launch / 配置 / 模型
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
        # 示例数据（少量，供离线复现）
        (os.path.join("share", package_name, "data", "sample"),
         glob("data/sample/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eeexty",
    maintainer_email="eeexty@users.noreply.github.com",
    description="MuJoCo 3.12 四旋翼仿真底座（ROS2 Humble）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 作业要求：入口以 main 开头
            "main = uav_mujoco_sim.cli:main",
            # 另保留短名字入口，便于单独调试某一层
            "bridge = uav_mujoco_sim.ros.bridge_node:main",
            "keyboard = uav_mujoco_sim.ros.keyboard_node:main",
        ],
    },
)
