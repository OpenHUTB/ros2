#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_python 打包配置。

本功能包既是 ROS 2 包，也可以脱离 ROS 直接运行：

* ROS 2 方式：``colcon build --packages-select autonomous_car_stanley_tracking``
* 独立方式：``python3 main.py --mode demo``（只需 numpy，绘图另需 matplotlib）
"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "autonomous_car_stanley_tracking"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 把离线实验与文档一并安装，方便在目标机复现效果图
        (os.path.join("share", package_name, "test"), glob("test/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenHUTB",
    maintainer_email="openhutb@example.com",
    description="Stanley 前轴横向偏差跟踪：无人车横向控制与跟踪性能评测",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stanley_tracking_node = stanley_tracking.stanley_node:main",
        ],
    },
)
