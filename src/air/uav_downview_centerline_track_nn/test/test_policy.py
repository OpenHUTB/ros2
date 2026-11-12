"""神经网络策略单元测试。"""

import numpy as np

from uav_downview_centerline_track_nn.policy import TrackPolicyNetwork


def test_output_shape_and_bounds():
    net = TrackPolicyNetwork()
    out = net.forward(np.array([0.5, -0.3, 0.4]))
    assert out.shape == (2,)
    assert np.all(np.abs(out) <= 1.0)


def test_weights_not_collapsed():
    net = TrackPolicyNetwork()
    params = net.parameters()
    assert np.std(params["w1"]) > 0.0
    assert np.std(params["w2"]) > 0.0


def test_yaw_feedback_sign():
    """标线在左侧时应输出正偏航（向左转向标线）。"""
    net = TrackPolicyNetwork()
    out_left = net.forward(np.array([1.0, 0.0, 0.0]))
    assert out_left[1] > 0.0
