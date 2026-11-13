"""前沿打分网络测试。"""

import numpy as np

from uav_frontier_exploration_occupancy_nn.frontier_policy import FrontierScoreNetwork


def test_near_unknown_scores_higher():
    net = FrontierScoreNetwork()
    near_open = np.array([0.1, 0.0, 0.9, 0.5])
    far_closed = np.array([0.9, 0.0, 0.1, 0.5])
    assert net.score(near_open) > net.score(far_closed)


def test_forward_selects_argmax():
    net = FrontierScoreNetwork()
    candidates = [
        {"features": np.array([0.9, 0.0, 0.1, 0.5])},
        {"features": np.array([0.1, 0.0, 0.9, 0.5])},
        {"features": np.array([0.5, 0.3, 0.4, 0.5])},
    ]
    decision = net.forward(candidates)
    assert decision["index"] == 1
    assert len(decision["scores"]) == 3
