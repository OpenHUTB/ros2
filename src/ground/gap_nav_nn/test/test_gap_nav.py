"""狭窄通道通行的离线单测（不需要 ROS，不需要 MuJoCo 场景运行）。

运行：
    cd src/ground/gap_nav_nn
    python3 -m pytest test/ -q
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

import oracle                                          # noqa: E402
from nn import MLP                                      # noqa: E402


GAPS = [{"lo": -3.55, "hi": -2.85},                     # 0.70 m，不可通行
        {"lo": -0.70, "hi": 0.70},                      # 1.40 m
        {"lo": 2.00, "hi": 4.40}]                       # 2.40 m


def test_passable_gaps_filters_narrow_opening():
    got = oracle.passable_gaps(GAPS, min_passable=0.95)
    assert len(got) == 2
    assert all(g["width"] >= 0.95 for g in got)


def test_min_passable_changes_result():
    # 抬高门槛后只剩最宽的开口
    got = oracle.passable_gaps(GAPS, min_passable=2.0)
    assert len(got) == 1
    assert abs(got[0]["width"] - 2.4) < 1e-9


def test_choose_gap_prefers_short_detour():
    # 起点和目标都在下方 -> 应选 1.4 m 的下开口（中心 0），而不是 2.4 m 的上开口
    g = oracle.choose_gap((-6.0, -1.0), (6.0, -1.5), GAPS, min_passable=0.95)
    assert g is not None
    assert abs(g["center"] - 0.0) < 1e-9


def test_choose_gap_returns_none_when_nothing_passable():
    assert oracle.choose_gap((-6.0, 0.0), (6.0, 0.0), GAPS, min_passable=9.0) is None


def test_build_path_goes_through_chosen_gap_center():
    path, gap = oracle.build_path((-6.0, -1.0), (6.0, -1.5), GAPS, min_passable=0.95)
    assert gap is not None
    # 折线里应有 x ≈ 0 且 y ≈ 开口中心的点
    crossed = path[np.abs(path[:, 0]) < 0.05]
    assert crossed.shape[0] >= 1
    assert np.min(np.abs(crossed[:, 1] - gap["center"])) < 0.05


def test_mlp_forward_shape_and_determinism():
    net = MLP([6, 16, 16, 2], seed=1)
    x = np.random.default_rng(0).normal(size=(4, 6))
    y1 = net.predict(x)
    y2 = net.predict(x)
    assert y1.shape == (4, 2)
    assert np.allclose(y1, y2)          # 前向是确定性的


def test_gap_expert_turns_toward_target():
    # 车在原点朝 +x，目标点在左上方 -> 角速度应为正（左转）
    path = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]], dtype=np.float64)
    ex = oracle.GapExpert(path)
    v, w = ex.act((0.0, 0.0, 0.0), 0.0, 0.0, scan=None)
    assert v > 0.0
    assert w > 0.0
