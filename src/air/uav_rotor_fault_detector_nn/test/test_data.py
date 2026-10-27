"""故障脚本、数据集存取与划分的单元测试。

涉及真实仿真（MuJoCo + 底座）的用例在没有底座时会自动跳过。
"""

import numpy as np
import pytest

from uav_rotor_fault_detector_nn.data import (
    FAULT_KINDS,
    GAIN_FLOOR_PROGRESSIVE,
    PHYSICAL_SCENES,
    ROTOR_FAULT_KINDS,
    SENSOR_FAULT_KINDS,
    FaultPlan,
    build_dataset,
    load_dataset,
    random_plan,
    save_dataset,
    simulate_episode,
    split_by_episode,
)


# ---------------------------------------------------------------- 故障脚本
def test_healthy_plan_keeps_all_gains():
    plan = FaultPlan("healthy")
    for t in (0.0, 1.9, 2.0, 5.0):
        np.testing.assert_allclose(plan.gains(t), np.ones(4))


def test_loss_drops_to_zero_after_onset():
    plan = FaultPlan("loss", rotor=2, onset=2.0, severity=1.0)
    np.testing.assert_allclose(plan.gains(1.99)[2], 1.0)
    np.testing.assert_allclose(plan.gains(2.0)[2], 0.0)
    # 其他桨不受影响
    np.testing.assert_allclose(np.delete(plan.gains(3.0), 2), np.ones(3))


def test_degradation_keeps_partial_thrust():
    plan = FaultPlan("degradation", rotor=0, onset=1.0, severity=0.4)
    assert plan.gains(0.5)[0] == pytest.approx(1.0)
    assert plan.gains(2.0)[0] == pytest.approx(0.6)


def test_stuck_zeroes_gain():
    plan = FaultPlan("stuck", rotor=3, onset=1.0)
    assert plan.gains(2.0)[3] == pytest.approx(0.0)


def test_dual_loss_affects_two_rotors():
    plan = FaultPlan("dual_loss", rotor=0, rotor2=2, onset=1.0)
    gains = plan.gains(2.0)
    assert gains[0] == pytest.approx(0.0)
    assert gains[2] == pytest.approx(0.0)
    np.testing.assert_allclose(gains[[1, 3]], np.ones(2))


def test_progressive_decays_with_rate():
    plan = FaultPlan("progressive", rotor=1, onset=2.0, severity=0.25)
    assert plan.gains(2.0)[1] == pytest.approx(1.0)
    assert plan.gains(4.0)[1] == pytest.approx(0.5)
    # 掉到地板值之后不再下降
    assert plan.gains(20.0)[1] == pytest.approx(GAIN_FLOOR_PROGRESSIVE)


def test_ramp_smears_the_onset():
    plan = FaultPlan("loss", rotor=0, onset=2.0, severity=1.0, ramp=1.0)
    assert plan.gains(2.0)[0] == pytest.approx(1.0)
    assert plan.gains(2.5)[0] == pytest.approx(0.5)
    assert plan.gains(3.0)[0] == pytest.approx(0.0)


def test_sensor_factor_by_kind():
    bias = (0.05, -0.02, 0.01)
    assert FaultPlan("imu_bias", gyro_bias=bias).sensor_factor() == (bias, 1.0)
    assert FaultPlan("imu_noise", gyro_bias=bias).sensor_factor() == (bias, 6.0)
    assert FaultPlan("loss").sensor_factor() == ((0.0, 0.0, 0.0), 1.0)


def test_describe_mentions_rotor():
    assert "桨" in FaultPlan("loss", rotor=1).describe()
    assert "桨" in FaultPlan("dual_loss", rotor=0, rotor2=2).describe()
    assert "健康" in FaultPlan("healthy").describe()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kind": "不存在"},
        {"kind": "loss", "rotor": 4},
        {"kind": "loss", "rotor": -1},
        {"kind": "loss", "onset": -0.1},
        {"kind": "loss", "ramp": -1.0},
        {"kind": "progressive", "severity": 0.0},
    ],
)
def test_invalid_plan_raises(kwargs):
    with pytest.raises(ValueError):
        FaultPlan(**kwargs)


def test_random_plan_is_valid():
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(200):
        plan = random_plan(rng)
        assert plan.kind in FAULT_KINDS
        assert 0 <= plan.rotor < 4
        assert plan.onset >= 0.0
        seen.add(plan.kind)
    assert seen == set(FAULT_KINDS)


def test_fault_kind_groups():
    assert "healthy" not in ROTOR_FAULT_KINDS
    assert "loss" in ROTOR_FAULT_KINDS
    assert set(SENSOR_FAULT_KINDS) == {"imu_bias", "imu_noise"}
    assert set(ROTOR_FAULT_KINDS) | set(SENSOR_FAULT_KINDS) | {"healthy"} == set(FAULT_KINDS)


# ---------------------------------------------------------------- 数据集
def _fake_dataset(n: int = 250, n_episodes: int = 10):
    per = n // n_episodes
    return {
        "X": np.zeros((n, 100, 13), dtype=np.float32),
        "Y": np.ones((n, 4), dtype=np.float32),
        "episode": np.repeat(np.arange(n_episodes), per).astype(np.int64),
        "end": np.arange(n, dtype=np.int64),
        "kinds": np.array(["loss"] * n_episodes, dtype=object),
        "scenes": np.array(list(PHYSICAL_SCENES)[:n_episodes], dtype=object) if n_episodes <= len(PHYSICAL_SCENES) else np.array(["open_ground"] * n_episodes, dtype=object),
        "crashed": np.zeros(n_episodes, dtype=bool),
        "crash_time": np.full(n_episodes, np.nan),
        "descriptions": np.array(["单桨完全失效(桨0)@2.0s"] * n_episodes, dtype=object),
        "window": np.array(100, dtype=np.int64),
        "stride": np.array(10, dtype=np.int64),
        "dt": np.array(0.01, dtype=np.float64),
    }


def test_dataset_roundtrip(tmp_path):
    dataset = _fake_dataset()
    path = tmp_path / "dataset.npz"
    save_dataset(path, dataset)
    loaded = load_dataset(path)

    np.testing.assert_allclose(loaded["X"], dataset["X"])
    np.testing.assert_allclose(loaded["Y"], dataset["Y"])
    np.testing.assert_array_equal(loaded["episode"], dataset["episode"])
    assert [str(v) for v in loaded["kinds"]] == [str(v) for v in dataset["kinds"]]
    assert [str(v) for v in loaded["descriptions"]] == [str(v) for v in dataset["descriptions"]]
    assert int(loaded["window"]) == 100


def test_split_by_episode_partitions_samples():
    dataset = _fake_dataset(n=250, n_episodes=10)
    masks = split_by_episode(dataset)
    total = sum(int(m.sum()) for m in masks.values())
    assert total == len(dataset["X"])
    # 任意两个集合不相交
    assert not np.any(masks["train"] & masks["val"])
    assert not np.any(masks["train"] & masks["test"])
    assert not np.any(masks["val"] & masks["test"])
    # 同一个 episode 不会跨集合
    for episode in np.unique(dataset["episode"]):
        hit = sum(int(np.any(masks[key] & (dataset["episode"] == episode))) for key in masks)
        assert hit == 1


def test_split_by_episode_rejects_bad_fractions():
    with pytest.raises(ValueError):
        split_by_episode(_fake_dataset(), fractions=(0.5, 0.1, 0.1))


# ---------------------------------------------------------------- 真实仿真
def _load_base_or_skip():
    pytest.importorskip("mujoco")
    from uav_rotor_fault_detector_nn._compat import load_base

    try:
        return load_base()
    except ImportError as exc:  # 底座不在时跳过而不是失败
        pytest.skip(str(exc))


def test_simulate_episode_shapes():
    api = _load_base_or_skip()
    plan = FaultPlan("loss", rotor=0, onset=2.0)
    episode = simulate_episode(api, plan, scene="open_ground", seed=0, duration=3.0)

    # stop_on_crash 默认开启，序列会在坠地那一刻截断，所以长度 <= 仿真时长 / dt
    assert 0 < episode.length <= 300
    assert episode.features.shape == (episode.length, 13)
    assert episode.gains.shape == (episode.length, 4)
    assert episode.altitude.shape == (episode.length,)
    assert episode.crashed
    assert 0.0 < episode.crash_time < 3.0
    # 故障注入前后的增益必须真的变化了
    assert episode.gains[0, 0] == pytest.approx(1.0)
    assert episode.gains[-1, 0] < 0.5


def test_healthy_episode_survives():
    api = _load_base_or_skip()
    episode = simulate_episode(api, FaultPlan("healthy"), scene="open_ground", seed=1, duration=3.0)
    assert not episode.crashed
    assert np.all(np.isfinite(episode.features))
    assert episode.altitude.min() > 0.5


def test_build_dataset_small():
    api = _load_base_or_skip()
    dataset = build_dataset(
        api, n_episodes=2, seed=0, scenes=("open_ground",), duration=2.0, window=50, stride=25
    )
    assert dataset["X"].shape[1:] == (50, 13)
    assert dataset["X"].shape[0] == dataset["Y"].shape[0] == len(dataset["end"])
    assert len(dataset["kinds"]) == 2
    assert dataset["window"] == 50
