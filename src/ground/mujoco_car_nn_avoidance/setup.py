# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "mujoco_car_nn_avoidance"

# 版本号只维护一处：从包内 __init__.py 读取，避免 setup.py 与包版本不一致
_init_text = Path(package_name, "__init__.py").read_text(encoding="utf-8")
_version = re.search(r'__version__\s*=\s*"([^"]+)"', _init_text).group(1)

setup(
    name=package_name,
    version=_version,
    packages=[package_name],
    data_files=[
        # ament 索引，缺少它 ros2 run / ros2 launch 找不到本功能包
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
        # 训练好的 MLP 权重随包安装，ROS 推理节点从这里加载
        (os.path.join("share", package_name, "results"), ["results/mlp_weights.npz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zh070507",
    maintainer_email="zh070507@example.com",
    description="MuJoCo 地面差速小车神经网络端到端避障：25 束射线 + 航向 -> 纯 numpy MLP 转向/油门",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_car_nn_avoidance_node = mujoco_car_nn_avoidance.ros_node:main",
            "mujoco_car_nn_sim_bridge_node = mujoco_car_nn_avoidance.sim_bridge:main",
        ],
    },
)
