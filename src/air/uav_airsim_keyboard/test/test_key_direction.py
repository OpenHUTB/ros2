# -*- coding: utf-8 -*-
"""按键映射与合成逻辑的单元测试（不需要仿真器）。

运行：python3 -m pytest test/test_key_direction.py -q
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_airsim_keyboard.keymap import KEY_DIRECTION, combine_keys  # noqa: E402


def test_key_table_covers_six_axes():
    assert set(KEY_DIRECTION) == {'w', 's', 'a', 'd', 'r', 'f', 'q', 'e'}
    assert KEY_DIRECTION['w'] == (1.0, 0.0, 0.0, 0.0)
    assert KEY_DIRECTION['q'][3] == 1.0 and KEY_DIRECTION['e'][3] == -1.0


def test_single_key_uses_full_speed():
    vx, vy, vz, wz = combine_keys(['w'], 1.0, 0.5)
    assert (vx, vy, vz) == (1.0, 0.0, 0.0)
    assert wz == 0.0


def test_diagonal_is_normalized():
    vx, vy, vz, wz = combine_keys(['w', 'd'], 1.0, 0.5)
    assert abs(math.hypot(vx, vy) - 1.0) < 1e-9


def test_opposite_keys_cancel():
    vx, vy, vz, wz = combine_keys(['w', 's'], 1.0, 0.5)
    assert abs(vx) < 1e-9 and abs(vy) < 1e-9


def test_vertical_and_yaw_combine():
    vx, vy, vz, wz = combine_keys(['r', 'q'], 1.0, 0.5)
    assert vz == 1.0 and wz == 0.5
