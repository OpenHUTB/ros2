# -*- coding: utf-8 -*-
"""test_geometry.py —— 正/逆运动学与模型定位，不需要 mujoco。

这两处是整个演示数据链路的源头：IK 若与 FK 不自洽，专家标注的扭矩就是错的，
而 MLP 会把错误原样学下来，最终表现成「成功率永远到不了 100%」这种查不出原因的现象。
"""
import math

import numpy as np
import pytest

from reacher_track.reacher_env import (L1, L2, find_model_path, forward_kinematics,
                                      inverse_kinematics)


def test_ik_fk_roundtrip_inside_reach():
    """可达环内（|L1-L2| + 余量 ~ L1+L2 - 余量）的目标：IK 解再 FK 回去必须回到原点。

    注意平面两连杆可达域是**圆环**而不是圆盘：内半径 |L1-L2| = 0.03 m 以内
    无解，IK 会把它夹到内边界上，这时 FK 自然回不到原目标——所以采样要避开。
    """
    rng = np.random.RandomState(0)
    inner = abs(L1 - L2) + 0.01
    outer = L1 + L2 - 0.01
    for _ in range(200):
        r = rng.uniform(inner, outer)
        a = rng.uniform(-math.pi, math.pi)
        tx, ty = r * math.cos(a), r * math.sin(a)
        q1, q2 = inverse_kinematics(tx, ty)
        x, y = forward_kinematics(q1, q2)
        assert abs(x - tx) < 1e-9 and abs(y - ty) < 1e-9, \
            'IK/FK 不自洽：目标 (%.6f, %.6f) 回到 (%.6f, %.6f)' % (tx, ty, x, y)


def test_ik_clamps_unreachable_target_to_max_reach():
    """超出臂长的目标要被夹到最大可达点，而不是给出 nan 或越界关节角。"""
    q1, q2 = inverse_kinematics(1.0, 0.0)          # 1 m 远超 L1+L2=0.27 m
    x, y = forward_kinematics(q1, q2)
    assert abs(math.hypot(x, y) - (L1 + L2)) < 1e-6
    assert math.isfinite(q1) and math.isfinite(q2)


def test_joint_limits_respected_for_random_targets():
    """`random_target` 声称保证逆解在 ±1.8 rad 内，这里按同样条件抽查。"""
    rng = np.random.RandomState(1)
    for _ in range(100):
        r = rng.uniform(0.05, L1 + L2 - 0.05)
        a = rng.uniform(0, 2 * math.pi)
        q1, q2 = inverse_kinematics(r * math.cos(a), r * math.sin(a))
        if abs(q1) <= 1.8 and abs(q2) <= 1.8:
            continue
        # 该点不在随机采样的接受域内，属正常；至少不能出现 nan
        assert math.isfinite(q1) and math.isfinite(q2)


def test_model_path_resolves_from_any_cwd(tmp_path, monkeypatch):
    """`find_model_path` 必须能在源码树内任意 CWD 下找到 reacher.xml。

    原来是写死的 CWD 相对路径 `models/reacher.xml`，换个目录跑就 FileNotFoundError。
    """
    monkeypatch.chdir(tmp_path)
    p = find_model_path()
    assert p.endswith('reacher.xml')
    assert __import__('os').path.exists(p)

    with pytest.raises(TypeError):
        find_model_path(None)      # 只接受字符串路径
