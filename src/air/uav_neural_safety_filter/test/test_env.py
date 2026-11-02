import numpy as np

from uav_neural_safety_filter.core.dataset import CollectConfig, collect
from uav_neural_safety_filter.core.env import (EnvConfig, SafetyEnv,
                                               make_episodes)
from uav_neural_safety_filter.core.experiment import run_episode
from uav_neural_safety_filter.core.filter import FilterConfig, NoFilter


def test_episode_shapes():
    specs = make_episodes([0], per_seed=1, n_obstacles=12)
    assert len(specs) == 1 and specs[0].start.shape == (3,)


def test_env_observe_step():
    env = SafetyEnv(EnvConfig(max_time=3.0, n_obstacles=12))
    obs = env.reset(make_episodes([0], per_seed=1, n_obstacles=12)[0])
    assert obs["ranges"].shape == (3, 36) and obs["feat"].shape == (116,)
    u = env.nominal_action()
    obs2, info, done = env.step(u)
    assert "collision" in info and obs2["feat"].shape == (116,)


def test_no_filter_runs():
    env = SafetyEnv(EnvConfig(max_time=3.0, n_obstacles=12))
    spec = make_episodes([1], per_seed=1, n_obstacles=12)[0]
    r = run_episode(env, NoFilter(), spec)
    assert r["steps"] > 0 and 0.0 <= r["progress"] <= 1.0


def test_collect_labels():
    d = collect(CollectConfig(seeds=[0], per_seed=1, max_frames=10,
                              max_time=2.0, verbose=False))
    assert len(d["h"]) > 0
    assert np.isfinite(d["h"]).all() and np.isfinite(d["q"]).all()
