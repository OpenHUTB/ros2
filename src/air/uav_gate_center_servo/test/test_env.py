import numpy as np
import pytest

pytest.importorskip("mujoco")

from uav_gate_center_servo.core.env import EnvConfig, GateServoEnv  # noqa: E402


@pytest.fixture(scope="module")
def env():
    e = GateServoEnv(EnvConfig())
    yield e
    e.close()


def _episode(env, action=None, seed=0, reset_kwargs=None):
    obs = env.reset(seed=seed, **(reset_kwargs or {}))
    done, info, k = False, {}, 0
    while not done and k < 300:
        a = env.oracle_command() if action is None else action
        obs, done, info = env.step(a)
        k += 1
    return info


def test_gate_geometry(env):
    assert abs(env.gate.x - 5.0) < 1e-6
    assert env.gate.half == 1.1
    assert env.gate.contains(env.gate.center)


def test_reset_ranges(env):
    for seed in range(10):
        o = env.reset(seed=seed)
        p = o["pos"]
        assert 1.99 <= env.gate.x - p[0] <= 6.01
        assert abs(p[1] - env.gate.y) <= 1.51
        assert abs(p[2] - env.gate.z) <= 0.81
        assert abs(float(o["yaw"])) < 0.05


def test_observation_contract(env):
    o = env.reset(seed=0)
    assert o["depth"].shape == (12, 16)
    assert o["depth"].dtype == np.float32
    assert o["label"].shape == (3,)
    assert np.isfinite(float(o["valid_frac"]))


def test_depth_is_not_blank(env):
    o = env.reset(seed=0)
    if float(o["depth"].max()) <= 0.0:
        pytest.skip("MuJoCo 深度渲染不可用（无 GPU/显示）")
    assert float(o["depth"].max()) > 0.5
    assert float(o["valid_frac"]) > 0.2


def test_pass(env):
    info = _episode(env, seed=0)
    assert info["reason"] == "pass"


def test_miss(env):
    info = _episode(env, action=np.array([1.0, 0.0, 0.0]),
                    reset_kwargs={"vertical": 1.5})
    assert info["reason"] == "miss"


def test_timeout():
    e = GateServoEnv(EnvConfig(max_time=0.1))
    try:
        info = _episode(e, action=np.zeros(3))
        assert info["reason"] == "timeout"
    finally:
        e.close()


def test_action_units(env):
    env.reset(seed=0)
    start = env._ref.copy()
    for _ in range(40):
        env._command(np.array([1.0, 0.0, 0.0]))
    speed = (env._ref[0] - start[0]) / (40 * env.cfg.frame_dt)
    assert 0.8 < speed < 1.1
