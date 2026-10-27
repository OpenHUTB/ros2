"""检查传感器、物理量和终止条件."""
import numpy as np
import pytest
from stable_baselines3.common.env_checker import check_env

from uav_vertical_ppo.environment import VerticalHoverEnv


def test_gym_contract_and_seed():
    env = VerticalHoverEnv()
    try:
        check_env(env, warn=True)
        a, _ = env.reset(seed=10)
        b, _ = env.reset(seed=10)
        np.testing.assert_allclose(a, b)
        assert a[0] + env.target == pytest.approx(env.data.qpos[0], abs=1e-6)
    finally:
        env.close()


def test_thrust_and_payload():
    env = VerticalHoverEnv('payload')
    try:
        env.reset(seed=1)
        assert env.model.body_mass[1] == pytest.approx(1.15)
        for action in (-100, 100):
            _, _, _, _, info = env.step([action])
            assert 0 <= info['thrust_n'] <= 20
        with pytest.raises(ValueError):
            env.step([float('nan')])
        with pytest.raises(ValueError):
            env.step([0, 0])
    finally:
        env.close()


def test_gust_is_removed_after_window():
    env = VerticalHoverEnv('gust')
    try:
        env.reset(seed=1)
        env.data.time = 2.1
        env.step([0])
        assert env.data.xfrc_applied[1, 2] == -1.5
        env.data.time = 3.1
        env.step([0])
        assert env.data.xfrc_applied[1, 2] == 0
    finally:
        env.close()
