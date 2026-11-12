#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""部署侧不依赖 gymnasium 的回归测试.

docs/air/uav_route_cruise.md 写明“只部署只需要 numpy”。但
``policy.build_observation`` 之前从 ``route_env`` 里 import 常量，而
``route_env`` 顶层 import gymnasium，导致只装了 numpy 的 ROS 机器上
节点第一次构造观测就 ModuleNotFoundError。
"""
import sys
from pathlib import Path

import numpy as np

SCRIPTS = (Path(__file__).resolve().parents[1]
           / "src" / "air" / "uav_route_cruise" / "scripts")
sys.path.insert(0, str(SCRIPTS))

import policy  # noqa: E402
from route_constants import N_SECTORS  # noqa: E402


def test_build_observation_without_gymnasium():
    obs = policy.build_observation(
        np.ones(N_SECTORS, dtype=np.float32),
        0.0, 0.0,
        np.array([4.0, 0.0], dtype=np.float32),
        1.0, 0.0,
    )
    assert obs.shape == (policy.OBS_DIM,)
    assert np.isfinite(obs).all()


def test_observation_layout_constants():
    assert N_SECTORS == 24
    assert policy.OBS_DIM == N_SECTORS + 6
