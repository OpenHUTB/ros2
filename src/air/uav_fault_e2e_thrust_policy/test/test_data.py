"""数据管线（DAgger 数据集、拼帧、切分）的单元测试。

纯 numpy，不需要 torch / mujoco。
"""

import numpy as np

from uav_fault_e2e_thrust_policy import data as D
from uav_fault_e2e_thrust_policy import policy as P
from uav_fault_e2e_thrust_policy.obs import DEPTH_SIZE, PROPRIO_DIM


def _fake_dataset(n: int = 9, episodes=(0, 0, 0, 1, 1, 2, 2, 2, 2), seed: int = 0):
    rng = np.random.default_rng(seed)
    return {
        "depth": rng.uniform(0.0, 1.0, size=(n, 1, DEPTH_SIZE, DEPTH_SIZE)).astype(np.float32),
        "proprio": rng.normal(size=(n, PROPRIO_DIM)).astype(np.float32),
        "label": rng.uniform(0.0, 1.0, size=(n, 4)).astype(np.float32),
        "episode": np.asarray(episodes, dtype=int),
        "step": np.arange(n, dtype=int),
    }


def test_stack_frames_shape():
    ds = _fake_dataset()
    assert D.stack_frames(ds, stack=3).shape == (9, 3, DEPTH_SIZE, DEPTH_SIZE)


def test_stack_frames_last_channel_is_current():
    ds = _fake_dataset()
    stacked = D.stack_frames(ds, stack=3)
    assert np.allclose(stacked[:, -1], ds["depth"][:, 0])


def test_stack_frames_never_crosses_episode():
    ds = _fake_dataset()
    stacked = D.stack_frames(ds, stack=3)
    ep = ds["episode"]
    for i in range(len(ep)):
        start = i
        while start > 0 and ep[start - 1] == ep[i]:
            start -= 1
        for k in range(3):
            j = max(start, i - (3 - 1 - k))
            assert np.allclose(stacked[i, k], ds["depth"][j, 0])


def test_stack_frames_pads_with_first_frame_of_episode():
    ds = _fake_dataset()
    stacked = D.stack_frames(ds, stack=3)
    ep = ds["episode"]
    first = int(np.flatnonzero(ep == ep[0])[0])
    for k in range(2):
        assert np.allclose(stacked[first, k], ds["depth"][first, 0])


def test_split_by_episode_has_no_overlap():
    ds = _fake_dataset(n=20, episodes=tuple([0] * 5 + [1] * 5 + [2] * 5 + [3] * 5))
    masks = D.split_by_episode(ds, fractions=(0.8, 0.1, 0.1), seed=0)
    train, val, test = masks["train"], masks["val"], masks["test"]
    assert not np.any(train & val)
    assert not np.any(train & test)
    assert not np.any(val & test)
    assert np.array_equal(train | val | test, np.ones(20, dtype=bool))


def test_split_by_episode_keeps_episodes_together():
    ds = _fake_dataset(n=20, episodes=tuple([0] * 5 + [1] * 5 + [2] * 5 + [3] * 5))
    masks = D.split_by_episode(ds, fractions=(0.8, 0.1, 0.1), seed=0)
    ep = ds["episode"]
    for mask in masks.values():
        for e in np.unique(ep[mask]):
            assert np.all(mask[ep == e]), "同一个 episode 不能被切成两半"


def test_split_by_episode_handles_few_episodes():
    ds = _fake_dataset(n=3, episodes=(0, 1, 2))
    masks = D.split_by_episode(ds, fractions=(0.8, 0.1, 0.1), seed=0)
    assert sum(int(m.sum()) for m in masks.values()) == 3
    assert masks["train"].any()


def test_make_plan_defaults():
    plan = D.make_plan({"kind": "degradation", "rotor": 2, "severity": 0.4})
    assert plan.kind == "degradation"
    assert plan.rotor == 2
    assert plan.onset == D.DEFAULT_ONSET
    assert np.isclose(plan.gains(0.0)[2], 1.0)
    # degradation 的 severity 是"掉多少推力比例"，所以 0.4 → 增益 0.6
    assert np.isclose(plan.gains(5.0)[2], 0.6)


def test_make_plan_healthy_is_all_ones():
    plan = D.make_plan({"kind": "healthy"})
    assert np.allclose(plan.gains(0.0), 1.0)
    assert np.allclose(plan.gains(9.0), 1.0)


def test_train_cases_cover_expected_faults():
    kinds = {spec["kind"] for _, spec in D.TRAIN_CASES}
    assert {"healthy", "degradation", "stuck", "progressive"} <= kinds


def test_eval_cases_include_boundary():
    kinds = {spec["kind"] for _, spec in D.EVAL_CASES}
    assert "loss" in kinds, "单桨全失效要作为边界案例出现在评估里"


def test_save_and_load_roundtrip(tmp_path):
    ds = _fake_dataset()
    path = tmp_path / "ds.npz"
    D.save_dataset(path, ds, [])
    back = D.load_dataset(path)
    for key in ("depth", "proprio", "label", "episode", "step"):
        assert np.allclose(back[key], ds[key])


def test_policy_depth_stack_matches_arch():
    assert P.DEPTH_STACK == 3
    assert P.ARCHS == ("mlp", "cnn", "cnn_stack")
