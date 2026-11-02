"""离线单元测试：不需要 ROS，只需要 mujoco + numpy（约 5 秒跑完）。"""

import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from uav_nav.data import to_net_input
from uav_nav.env import D_MAX, DepthUavEnv, yaw_of
from uav_nav.net import DepthCNN
from uav_nav.policy import PolicyConfig, act, run_episode


@pytest.fixture(scope="module")
def env():
    return DepthUavEnv("slalom")


# ---------------------------------------------------------------- 环境与物理

def test_reset_gives_expected_pose(env):
    state = env.reset(pos=(0.0, 0.0, 1.5), yaw=0.0)
    assert np.allclose(state.pos, (0, 0, 1.5), atol=1e-3)
    assert abs(state.yaw) < 1e-6
    assert not state.crashed


def test_hover_holds_altitude(env):
    env.reset()
    for _ in range(40):                      # 2 s
        state = env.step(0.0, 0.0)
    assert abs(state.pos[2] - 1.5) < 0.05, state.pos[2]
    assert abs(state.vel[2]) < 0.05, state.vel[2]


def test_forward_command_moves_forward(env):
    env.reset()
    for _ in range(40):
        state = env.step(1.0, 0.0)
    assert state.pos[0] > 1.5
    assert abs(state.pos[1]) < 0.1


def test_yaw_rate_is_tracked(env):
    env.reset()
    for _ in range(20):                      # 1 s @ 0.5 rad/s
        state = env.step(0.0, 0.5)
    assert abs(state.yaw - 0.5) < 0.08, state.yaw


def test_yaw_of_quaternion():
    q = np.array([np.cos(0.3), 0.0, 0.0, np.sin(0.3)])
    assert abs(yaw_of(q) - 0.6) < 1e-6


def test_crash_is_detected(env):
    """直接朝柱子飞，必须触发碰撞标志（否则避障评价没有意义）。"""
    env.reset(scene="gate", pos=(3.0, 0.0, 1.5), yaw=0.0)
    env.model.geom_pos[env.post_ids[0]] = (6.0, 0.0, 1.5)   # 正前方放一根柱子
    crashed = False
    for _ in range(200):
        state = env.step(1.5, 0.0)
        if state.crashed:
            crashed = True
            break
    assert crashed


# ---------------------------------------------------------------- 传感器

def test_depth_shape_and_range(env):
    env.reset()
    depth = env.depth()
    assert depth.shape == (64, 64)
    assert depth.min() >= 0.0 and depth.max() <= D_MAX


def test_depth_reacts_to_obstacle(env):
    """同一位置：前方加柱子后，正前方扇区余量必须变小。"""
    env.reset(scene="gate", pos=(2.0, 0.0, 1.5), yaw=0.0)
    env.model.geom_pos[env.post_ids[0]] = (14.0, 8.0, 1.5)     # 移走所有柱子
    for gid in env.post_ids[1:]:
        env.model.geom_pos[gid] = (14.0, 8.0, 1.5)
    env.refresh()
    far = env.sector_clearance(env.depth())

    env.model.geom_pos[env.post_ids[0]] = (4.0, 0.0, 1.5)      # 正前方 2 m 放柱子
    env.refresh()
    near = env.sector_clearance(env.depth())
    assert near[1] < far[1], (near, far)


def test_net_input_is_pooled_clearance(env):
    env.reset()
    depth = env.depth()
    x = to_net_input(depth)
    assert x.shape == (32, 32)
    assert 0.0 <= x.min() and x.max() <= 1.0


# ---------------------------------------------------------------- 网络

def test_param_count_matches_formula():
    net = DepthCNN(in_hw=32, c1=6, c2=12, hidden=32, out=3)
    h1, h2 = 15, 7
    expected = (9 * 1 * 6 + 6) + (9 * 6 * 12 + 12) + (h2 * h2 * 12 * 32 + 32) + (32 * 3 + 3)
    assert net.config()["params"] == expected


def test_forward_backward_shapes(env):
    net = DepthCNN(seed=1)
    x = np.random.rand(4, 32, 32, 1).astype(np.float32)
    y = np.random.rand(4, 3).astype(np.float32)
    pred, cache = net.forward(x)
    assert pred.shape == (4, 3)
    assert pred.min() >= 0 and pred.max() <= 1
    grads = net.backward(cache, y)
    for key, value in grads.items():
        assert value.shape == net.p[key].shape, key
        assert np.isfinite(value).all(), key


def test_gradient_matches_finite_difference():
    net = DepthCNN(hidden=8, seed=2)
    rng = np.random.default_rng(0)
    x = rng.random((2, 32, 32, 1)).astype(np.float32)
    y = rng.random((2, 3)).astype(np.float32)
    _, cache = net.forward(x)
    grads = net.backward(cache, y)

    def loss():
        return float(np.mean(((net.predict(x) - y) ** 2).sum(axis=1)))

    eps = 1e-2
    for key in ("w4", "b4"):
        orig = net.p[key].copy()
        for idx in [(0, 0), (1, 1)] if orig.ndim == 2 else [(0,)]:
            base = orig[idx]
            net.p[key][idx] = base + eps
            plus = loss()
            net.p[key][idx] = base - eps
            minus = loss()
            net.p[key][idx] = base
            numeric = (plus - minus) / (2 * eps)
            assert abs(numeric - grads[key][idx]) < 2e-3 * max(1.0, abs(numeric))


def test_short_training_reduces_loss(env):
    from uav_nav.train import train

    net, history, (x_va, y_va) = train(env, epochs=3, n_train=60, n_val=20, lr=3e-3,
                                       log=lambda *a, **k: None)
    assert history[-1]["train_mse"] < history[0]["train_mse"]
    assert abs(float(np.abs(net.predict(x_va) - y_va).mean())) < 0.30


# ---------------------------------------------------------------- 策略

def test_act_outputs_are_bounded(env):
    net = DepthCNN(seed=0)
    env.reset()
    cfg = PolicyConfig()
    vx, yaw_rate, c = act(net, env.depth(), cfg, yaw=0.0)
    assert 0.0 <= vx <= cfg.v_max
    assert abs(yaw_rate) <= cfg.w_max
    assert c.shape == (3,) and c.min() >= 0 and c.max() <= 1


def test_heading_term_damps_turning(env):
    """同样的深度图, 偏航角越大, 回正项让偏航指令越小。"""
    net = DepthCNN(seed=0)
    env.reset()
    depth = env.depth()
    cfg = PolicyConfig()
    _, w0, _ = act(net, depth, cfg, yaw=0.0)
    _, w1, _ = act(net, depth, cfg, yaw=0.8)
    assert w1 < w0


def test_episode_runs_and_reports(env):
    net = DepthCNN(seed=0)
    cfg = PolicyConfig(steps=20)
    res = run_episode(net, "slalom", seed=0, cfg=cfg, env=env)
    assert res.steps <= 20
    assert 0.0 <= res.min_clearance <= 1.0
    assert res.mean_pred_error >= 0.0
