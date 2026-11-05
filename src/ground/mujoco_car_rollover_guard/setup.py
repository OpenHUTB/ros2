# -*- coding: utf-8 -*-
"""ament_python 功能包安装脚本。"""

import os
import re
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "mujoco_car_rollover_guard"

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
        (os.path.join("share", package_name, "results"), ["results/rollover_risk.npz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zh070507",
    maintainer_email="3316083292@qq.com",
    description="MuJoCo 地面差速小车侧倾/翻车风险预测与主动限速：9 维 IMU 特征 -> 纯 numpy MLP -> 风险门控",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_car_rollover_sim_bridge_node = mujoco_car_rollover_guard.sim_bridge:main",
            "mujoco_car_rollover_guard_node = mujoco_car_rollover_guard.guard_node:main",
        ],
    },
)
