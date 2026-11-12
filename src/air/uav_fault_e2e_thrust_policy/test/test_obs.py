"""观测构造的单元测试（不需要 torch / mujoco）。"""

import numpy as np
import pytest

from uav_fault_e2e_thrust_policy import obs as O


class _StubState:
    """只带 proprio_features 需要的字段。"""

    def __init__(self, vel=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0),
                 omega=(0.0, 0.0, 0.0), roll=0.0, pitch=0.0):
        self.vel = np.asarray(vel, dtype=float)
        self.quat = np.asarray(quat, dtype=float)
        self.omega = np.asarray(omega, dtype=float)
        self.roll = float(roll)
        self.pitch = float(pitch)


def test_crop_depth_pools_to_target_size():
    depth = np.random.default_rng(0).uniform(0.0, 10.0, size=(64, 64))
    small = O.crop_depth(depth, O.DEPTH_SIZE)
    assert small.shape == (O.DEPTH_SIZE, O.DEPTH_SIZE)
    assert small.dtype == np.float32


def test_crop_depth_accepts_already_small_input():
    depth = np.random.default_rng(3).uniform(0.0, 4.0, size=(O.DEPTH_SIZE, O.DEPTH_SIZE))
    small = O.crop_depth(depth, O.DEPTH_SIZE)
    assert small.shape == (O.DEPTH_SIZE, O.DEPTH_SIZE)
    assert np.all(np.isfinite(small))


def test_crop_depth_normalizes_by_max_depth():
    depth = np.full((64, 64), O.MAX_DEPTH)
    small = O.crop_depth(depth, O.DEPTH_SIZE)
    assert np.allclose(small, 1.0)
    near = np.zeros((64, 64))
    assert np.allclose(O.crop_depth(near, O.DEPTH_SIZE), 0.0)


def test_crop_depth_is_order_preserving():
    depth = np.zeros((64, 64))
    depth[:32] = 2.0       # 上半近
    depth[32:] = 8.0       # 下半远
    small = O.crop_depth(depth, O.DEPTH_SIZE)
    assert small[: O.DEPTH_SIZE // 2].mean() < small[O.DEPTH_SIZE // 2:].mean()


def test_body_velocity_rotates_world_velocity():
    quat = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])   # 偏航 90°
    vel_world = np.array([1.0, 0.0, 0.0])
    vel_body = O.body_velocity(vel_world, quat)
    assert vel_body.shape == (3,)
    assert np.isclose(np.linalg.norm(vel_body), 1.0)
    assert not np.allclose(vel_body, vel_world)


def test_proprio_features_shape_and_finiteness():
    state = _StubState(vel=(0.3, -0.1, 0.05), omega=(0.0, 0.0, 0.2))
    feat = O.proprio_features(state, np.array([1.0, 0.5, 1.0, 1.0]),
                              last_commands=np.full(4, 2.33))
    assert feat.shape == (O.PROPRIO_DIM,)
    assert np.all(np.isfinite(feat))
    assert 0.5 in feat                        # 健康度原样带进来


def test_proprio_features_uses_body_frame_velocity():
    gains = np.ones(4)
    a = O.proprio_features(_StubState(vel=(1.0, 0.0, 0.0)), gains)
    b = O.proprio_features(_StubState(vel=(0.0, 1.0, 0.0)), gains)
    assert not np.allclose(a, b)


def test_proprio_features_defaults_last_command():
    feat = O.proprio_features(_StubState(), np.ones(4), last_commands=None)
    assert feat.shape == (O.PROPRIO_DIM,)
    assert np.all(feat[-4:] > 0.0)


def test_proprio_names_match_dim():
    assert len(O.PROPRIO_NAMES) == O.PROPRIO_DIM


def test_build_observation_matches_declared_dims():
    depth = np.random.default_rng(1).uniform(0.0, 5.0, size=(64, 64))
    proprio = O.proprio_features(_StubState(), np.ones(4))
    ob = O.build_observation(depth, proprio)
    assert ob["depth"].shape == (1, O.DEPTH_SIZE, O.DEPTH_SIZE)
    assert ob["proprio"].shape == (O.PROPRIO_DIM,)
    flat = O.flatten_observation(ob)
    assert flat.shape == (O.OBS_DIM,)
    assert np.all(np.isfinite(flat))


def test_flatten_observation_stable_order():
    ob = {"depth": np.zeros((1, O.DEPTH_SIZE, O.DEPTH_SIZE), dtype=np.float32),
          "proprio": np.arange(O.PROPRIO_DIM, dtype=np.float32)}
    flat = O.flatten_observation(ob)
    assert np.allclose(flat[-O.PROPRIO_DIM:], np.arange(O.PROPRIO_DIM))


@pytest.mark.parametrize("size", [16, 32])
def test_crop_depth_supports_multiple_sizes(size):
    depth = np.random.default_rng(2).uniform(0.0, 3.0, size=(64, 64))
    assert O.crop_depth(depth, size).shape == (size, size)


def test_observation_has_no_world_position():
    """观测里不能有世界系位置/目标点（否则等于偷看 GPS）。"""
    names = " ".join(O.PROPRIO_NAMES).lower()
    assert "x_world" not in names and "pos" not in names and "goal" not in names
