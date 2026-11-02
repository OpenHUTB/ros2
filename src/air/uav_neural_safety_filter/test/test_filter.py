import numpy as np

from uav_mujoco_sim.core.scenes import Scene, SceneObject
from uav_neural_safety_filter.core.filter import (FilterConfig, GeometricFilter,
                                                  LearnedFilter, NoFilter,
                                                  _select, candidates)
from uav_neural_safety_filter.core.network import PolicyNet
from uav_neural_safety_filter.core.ranges import MAX_RANGE, N_AZ, N_EL


def _obs():
    return {"pos": np.array([0.0, 0.0, 1.0]), "vel": np.zeros(3), "yaw": 0.0,
            "ranges": np.full((N_EL, N_AZ), MAX_RANGE, np.float32),
            "h": np.float32(3.0), "feat": np.zeros(116, np.float32),
            "goal_dir": np.array([1.0, 0.0, 0.0], np.float32)}


def test_candidates_include_zero_and_clip():
    c = candidates(np.array([2.5, 0.0, 0.1]))
    assert np.any(np.all(np.abs(c) < 1e-9, axis=1))
    assert c[:, 0].max() <= 4.0 and c[:, 2].max() <= 2.0


def test_select_prefers_brake_when_infeasible():
    u = np.array([2.0, 0.0, 0.0])
    c = candidates(u)
    q = np.full(len(c), -1.0)
    uu, _ = _select(u, c, 0.5, q, FilterConfig())
    assert np.linalg.norm(uu) < 1e-9


def test_select_keeps_nominal_when_safe():
    u = np.array([2.0, 0.0, 0.0])
    c = candidates(u)
    q = np.full(len(c), 3.0)
    uu, _ = _select(u, c, 3.0, q, FilterConfig(progress=0.0, smooth=0.0))
    assert np.allclose(uu, u)


def test_geometric_filter_turns():
    sc = Scene(name="t", objects=[SceneObject("obstacle_box0", "box",
                                             (1.2, 0.0, 1.0), (0.5, 0.5, 1.0))])
    env = type("E", (), {"scene": sc})()
    f = GeometricFilter(env, FilterConfig(progress=0.0))
    u, _ = f.act(_obs(), np.array([2.0, 0.0, 0.0]))
    assert np.all(np.isfinite(u)) and not np.allclose(u, [2.0, 0.0, 0.0])


def test_learned_filter_finite():
    f = LearnedFilter(PolicyNet(hidden=8))
    u, _ = f.act(_obs(), np.array([2.0, 0.0, 0.0]))
    assert np.all(np.isfinite(u))


def test_no_filter_identity():
    u = np.array([1.0, 0.2, -0.1])
    assert np.allclose(NoFilter().act(_obs(), u)[0], u)
