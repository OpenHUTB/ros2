"""通行度策略网络单元测试。"""

import numpy as np

from uav_depth_terrain_passability_nn.policy import PassabilityNetwork


def test_output_shape_and_bounds():
    net = PassabilityNetwork()
    out = net.forward(np.zeros(9))
    assert out["passability"].shape == (3,)
    assert abs(float(np.sum(out["passability"])) - 1.0) < 1e-6
    assert -1.0 <= out["speed"] <= 1.0
    assert out["sector"] in (0, 1, 2)


def test_weights_not_collapsed():
    net = PassabilityNetwork()
    params = net.parameters()
    assert np.std(params["w1"]) > 0.0
    assert np.std(params["w_logits"]) > 0.0


def test_blocked_sector_lower_passability():
    """某扇区高度与台阶越高，其通行度应低于开放扇区。"""
    net = PassabilityNetwork()
    # 左扇区抬高，其余开放
    feat = np.zeros(9)
    feat[0] = 1.0
    feat[1] = 1.0
    open_case = net.forward(np.zeros(9))
    blocked_case = net.forward(feat)
    assert blocked_case["passability"][0] < open_case["passability"][0]
