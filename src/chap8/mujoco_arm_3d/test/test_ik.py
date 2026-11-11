#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK 精度回归测试（pytest 入口）。

用法（在包目录 src/chap8/mujoco_arm_3d 下执行）：
    pytest test -q
    python3 test/test_ik.py

被测逻辑在 verify_arm3d.py 的 run_ik()：螺旋线 8 个空间点，
解析逆解 -> 写入 qpos -> MuJoCo 引擎正解读数。
这条路径只算运动学、不做渲染，不需要 GL 上下文，无显卡 / 无 DISPLAY 的机器也能跑。
阈值 0.01 mm 与 verify_arm3d.py 保持一致。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))      # 让 import verify_arm3d 找到包根目录

import verify_arm3d  # noqa: E402


def test_ik_precision():
    """8 个螺旋点逆解 -> 引擎正解回代，最大误差应小于 0.01 mm。"""
    assert verify_arm3d.run_ik() == 0


if __name__ == "__main__":
    sys.exit(0 if verify_arm3d.run_ik() == 0 else 1)
