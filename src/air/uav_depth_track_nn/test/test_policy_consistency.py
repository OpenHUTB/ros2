"""回归测试：训练标签与推理公式必须一致。

采集端写进 CSV 的是专家控制律给出的**完整动作** ``(vx, vy)``；
推理端若把网络输出再叠加同一条几何律，专家分量就被算了两遍。
这里用三条断言把这个约定固定下来。
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from uav_depth_track_nn.model import blend_command, track_features  # noqa: E402


def test_default_gain_returns_network_output():
    """默认 gain=0：最终指令必须等于网络输出本身（只做限幅）。"""
    net = (0.42, -0.31)
    features = (0.6, -0.2, 0.5, 0.8)
    vx, vy = blend_command(net, features)
    assert abs(vx - 0.42) < 1e-9
    assert abs(vy + 0.31) < 1e-9


def test_gain_adds_exactly_one_geometric_term():
    """gain=g 时，几何项与网络项各出现一次，而不是两次。"""
    net = (0.0, 0.0)
    gain = 1.4
    features = (1.0, 0.5, 0.0, 1.0)
    vx, vy = blend_command(net, features, gain=gain, max_speed=10.0)
    assert abs(vx - gain * 5.0 * 1.0) < 1e-9
    assert abs(vy - gain * 5.0 * 0.5) < 1e-9


def test_clip_keeps_direction():
    """限幅只按比例缩放，不改变方向。"""
    net = (3.0, 4.0)
    vx, vy = blend_command(net, (0.0, 0.0, 0.0, 1.0), max_speed=1.0)
    assert abs(math.hypot(vx, vy) - 1.0) < 1e-6
    assert abs(vx / vy - 0.75) < 1e-9


def test_features_match_expert_law():
    """CSV 里的 vx/vy 就是几何律输出，说明标签是完整动作而非残差。"""
    yaw = 0.0
    pose = (0.0, 0.0)
    target = (2.0, 1.0)
    features = track_features(pose, yaw, target)
    # 专家律 vx = gain * 5 * f1，取 gain=1.0 时应还原出机体系下的偏差
    vx, vy = blend_command((0.0, 0.0), features, gain=1.0, max_speed=10.0)
    assert abs(vx - 2.0) < 1e-6
    assert abs(vy - 1.0) < 1e-6


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'ok  {name}')
    print('全部通过')
