# -*- coding: utf-8 -*-
"""catkin python 打包：安装 main.py 入口 + 它依赖的全部模块，
保证从安装空间启动时能找到 sensor_fusion_core / fusion_perception 等依赖。"""
from setuptools import setup

setup(
    name="carla_sensor_fusion",
    version="0.1.0",
    description="多传感器融合跟踪模块统一入口",
    # 入口脚本
    scripts=["main.py"],
    # 依赖模块（与 main.py 同目录，安装后进入 site-packages）
    py_modules=[
        "sensor_fusion_core",
        "fusion_perception",
        "sim_2d_fusion",
        "carla_sensor_fusion",
        "test_sensor_fusion",
        "offline_replay",
    ],
    install_requires=["numpy", "matplotlib"],
)
