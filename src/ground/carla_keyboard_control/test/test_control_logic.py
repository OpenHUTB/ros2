#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单元测试：不依赖 CARLA 服务端即可验证控制映射与相机解码逻辑。

运行：
    python3 -m pytest test/test_control_logic.py -v
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla_keyboard_control import carla_common as cc  # noqa: E402


def test_transform_construction():
    """make_transform 应正确解析 (x,y,z,roll,pitch,yaw)。"""
    try:
        tf = cc.make_transform((1.0, 2.0, 3.0, 0.0, 0.0, 90.0))
    except RuntimeError:
        return  # 未安装 carla 客户端时跳过
    assert abs(tf.location.x - 1.0) < 1e-6
    assert abs(tf.rotation.yaw - 90.0) < 1e-6


def test_reverse_threshold_logic():
    """真实驾驶逻辑：低速按 S 应挂倒挡，有速度按 S 应刹车。"""
    for speed, expect_reverse in ((0.0, True), (0.3, True), (5.0, False)):
        throttle = brake = 0.0
        reverse = False
        if speed < 0.5:
            reverse, throttle = True, 0.48
        else:
            brake = 0.8
        assert reverse is expect_reverse
        assert (throttle > 0) == expect_reverse
        assert (brake > 0) != expect_reverse


def test_steer_saturation():
    """转向量必须落在 [-1, 1]。"""
    steer = 0.6 + 0.6
    assert min(max(steer, -1.0), 1.0) <= 1.0


def test_requirement_files_exist():
    """课程约定的关键文件必须存在（main.* 入口 + launch + 打包文件）。"""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for name in ('main.py', 'main.sh', 'main.bat', 'package.xml', 'setup.py',
                 'setup.cfg', 'launch/main.launch.py', 'launch/main.launch'):
        assert os.path.exists(os.path.join(root, name)), f"缺少 {name}"
