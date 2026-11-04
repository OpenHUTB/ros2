#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置。

本模块**不依赖 ROS**：`ros2 run` / `ros2 launch` 只是"多一种启动方式"。
因此 `data_files` 里同时安装 launch、模型 XML 与说明文档，保证安装后
`ros2 launch mujoco_balance_car main.launch.py` 能找到模型。
"""

from glob import glob

from setuptools import setup

package_name = "mujoco_balance_car"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        # 模型 XML 装到 share 里，安装后 main.py 仍能按绝对路径找到
        ("share/" + package_name, glob("*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB",
    maintainer_email="openhutb@example.com",
    description="MuJoCo 两轮自平衡小车（双环 PID）与离线指标评测",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_balance_car = mujoco_balance_car.cli:main",
        ],
    },
)
