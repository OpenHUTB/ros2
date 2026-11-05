# -*- coding: utf-8 -*-
"""catkin python 打包：注册 main.py 为可执行入口，支持 rosrun/roslaunch。"""
from setuptools import setup

setup(
    name="carla_sensor_fusion",
    version="0.1.0",
    description="多传感器融合跟踪模块统一入口",
    scripts=["main.py"],
    install_requires=["numpy", "matplotlib"],
)
