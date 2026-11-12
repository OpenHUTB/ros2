#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""随包入库资源路径解析（只依赖 os + 可选 ament，无 carla/cv2 依赖）。

单独成模块是为了让 CI（无 CARLA 客户端）也能导入并断言权重路径存在，
真正起到防回归作用——见 test_carla_nn_module.TestShippedWeights。
"""
import os

# PilotNet 权重：优先取安装空间 share 目录（ros2 run 走 install 空间），
# 回退到模块根目录（源码空间，与 setup.py / package.xml / models/ 同级）
try:
    from ament_index_python.packages import get_package_share_directory
    _ROOT = get_package_share_directory('carla_nn_perception_control_slam')
except Exception:
    _ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.path.join(_ROOT, 'models', 'pilotnet.pt')
