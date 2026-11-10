# -*- coding: utf-8 -*-
"""render_gif 的 MuJoCo 分支回归测试。

不依赖真实 OpenGL：CartPoleEnv 使用真实 MuJoCo 模型，render_frame 换成
返回固定灰度帧的假函数。修复前 ``_set_env_state`` 里的 ``mujoco`` 没有
导入，NameError 会被 ``_try_mujoco_frame`` 的 except 吞掉，返回 ok=False，
于是渲染永远静默退回 matplotlib。
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from rl_algorithm_compare.cartpole_env import CartPoleEnv
from rl_algorithm_compare.render_gif import _try_mujoco_frame


def test_mujoco_frame_is_rendered_not_silently_skipped():
    env = CartPoleEnv()
    env.render_frame = lambda: np.full((48, 64, 3), 128, dtype=np.uint8)

    frame, ok = _try_mujoco_frame(env, True, [0.1, 0.2, 0.3, 0.4])

    assert ok, "MuJoCo 分支被异常吞掉，静默退回了 matplotlib"
    assert frame.shape == (48, 64, 3)
