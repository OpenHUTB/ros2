"""Tests for episode-safe slip learning and inference."""

import json
from pathlib import Path

import numpy as np
import pytest

from mujoco_diff_drive_slip_compensation.dataset import (
    DatasetSample,
    EpisodeCsvWriter,
    assign_episode_split,
)
from mujoco_diff_drive_slip_compensation.generate_sample_dataset import (
    generate_dataset,
)
from mujoco_diff_drive_slip_compensation.slip_model import (
    FEATURE_NAMES,
    TemporalSlipMlp,
    load_episodes,
    make_windows,
    read_episode,
    row_features,
    train_model,
)


def _episode_id(split: str, index: int) -> str:
    suffix = index
    while True:
        episode_id = f'mlp_{split}_{suffix:04d}'
        if assign_episode_split(episode_id, 25012) == split:
            return episode_id
        suffix += 1


def _write_episode(directory: Path, split: str, index: int, count: int = 32):
    episode_id = _episode_id(split, index)
    writer = EpisodeCsvWriter(
        directory,
        episode_id,
        25012,
        metadata={'mujoco_version': '3.12.0', 'random_seed': 25012},
    )
    slip = 0.05 + 0.02 * (index % 5)
    for frame in range(count):
        command = 0.3 + 0.05 * np.sin(frame / 4.0)
        wheel_speed = command / (0.08 * (1.0 - slip))
        sample = DatasetSample(
            timestamp_ns=frame * 20_000_000,
            episode_id=episode_id,
            split=writer.split,
            friction_coefficient=0.25 + 0.1 * (index % 4),
            command_linear_x=float(command),
            command_angular_z=float(0.1 * np.cos(frame / 5.0)),
            wheel_position_left_front=frame * wheel_speed * 0.02,
            wheel_position_left_rear=frame * wheel_speed * 0.02,
            wheel_position_right_front=frame * wheel_speed * 0.02,
            wheel_position_right_rear=frame * wheel_speed * 0.02,
            wheel_velocity_left_front=float(wheel_speed),
            wheel_velocity_left_rear=float(wheel_speed),
            wheel_velocity_right_front=float(wheel_speed),
            wheel_velocity_right_rear=float(wheel_speed),
            imu_acceleration_x=float(slip),
            imu_acceleration_y=0.0,
            imu_acceleration_z=9.81,
            imu_angular_velocity_x=0.0,
            imu_angular_velocity_y=0.0,
            imu_angular_velocity_z=0.0,
            ground_truth_x=0.0,
            ground_truth_y=0.0,
            ground_truth_yaw=0.0,
            ground_truth_linear_x=float(command),
            ground_truth_angular_z=float(0.1 * np.cos(frame / 5.0)),
        )
        writer.write(sample)
    writer.close()
    return episode_id


def test_row_features_builds_slip_target_from_encoder_and_truth():
    row = {
        'wheel_velocity_left_front': '10',
        'wheel_velocity_left_rear': '10',
        'wheel_velocity_right_front': '10',
        'wheel_velocity_right_rear': '10',
        'command_linear_x': '0.8',
        'command_angular_z': '0.0',
        'imu_acceleration_x': '0.1',
        'imu_acceleration_y': '0.0',
        'imu_acceleration_z': '9.81',
        'imu_angular_velocity_x': '0.0',
        'imu_angular_velocity_y': '0.0',
        'imu_angular_velocity_z': '0.0',
        'ground_truth_linear_x': '0.72',
    }

    features, slip = row_features(row)

    assert features.shape == (len(FEATURE_NAMES),)
    assert slip == pytest.approx(0.1)


def test_windows_never_join_adjacent_episodes(tmp_path: Path):
    first = _write_episode(tmp_path, 'train', 1, count=5)
    second = _write_episode(tmp_path, 'train', 20, count=6)
    episodes = load_episodes(tmp_path)

    windows = make_windows(episodes, window_size=4)

    assert set(windows.episode_ids) == {first, second}
    assert windows.episode_ids.count(first) == 2
    assert windows.episode_ids.count(second) == 3
    assert len(windows.targets) == 5


def test_episode_reader_rejects_manifest_and_csv_split_mismatch(tmp_path: Path):
    _write_episode(tmp_path, 'train', 10)
    manifest_path = next(tmp_path.glob('*.json'))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['split'] = 'test'
    manifest_path.write_text(json.dumps(manifest), encoding='utf-8')

    with pytest.raises(ValueError, match='disagrees'):
        read_episode(manifest_path.with_suffix('.csv'))


def test_loader_rejects_duplicate_episode_ids(tmp_path: Path):
    _write_episode(tmp_path, 'train', 41)
    first_csv = next(tmp_path.glob('*.csv'))
    duplicate_csv = tmp_path / 'duplicate.csv'
    duplicate_json = duplicate_csv.with_suffix('.json')
    duplicate_csv.write_bytes(first_csv.read_bytes())
    duplicate_json.write_bytes(first_csv.with_suffix('.json').read_bytes())

    with pytest.raises(ValueError, match='duplicate episode ID'):
        load_episodes(tmp_path)


def test_training_is_seeded_saves_loadable_model_and_metrics(tmp_path: Path):
    data_dir = tmp_path / 'episodes'
    data_dir.mkdir()
    for split, start in (('train', 100), ('validation', 200), ('test', 300)):
        for index in range(3):
            _write_episode(data_dir, split, start + index, count=24)

    first_output = tmp_path / 'models-a'
    second_output = tmp_path / 'models-b'
    first = train_model(data_dir, first_output, window_size=6, epochs=24, seed=91)
    second = train_model(data_dir, second_output, window_size=6, epochs=24, seed=91)

    model = TemporalSlipMlp.load(first_output / 'slip_model.json')
    assert first['episodes'] == second['episodes']
    assert first['mae'] == pytest.approx(second['mae'])
    assert first['rmse'] == pytest.approx(second['rmse'])
    assert first['slip_f1'] >= 0.0
    assert first['inference_latency_median_ms'] > 0.0
    assert (first_output / 'training_metrics.json').is_file()
    assert model.predict(np.zeros((6, len(FEATURE_NAMES)))).__class__ is float


def test_mlp_rejects_wrong_window_shape():
    model = TemporalSlipMlp(
        window_size=2,
        hidden_size=3,
        mean=np.zeros(len(FEATURE_NAMES)),
        scale=np.ones(len(FEATURE_NAMES)),
        w1=np.zeros((2 * len(FEATURE_NAMES), 3)),
        b1=np.zeros(3),
        w2=np.zeros((3, 1)),
        b2=np.zeros(1),
    )

    with pytest.raises(ValueError, match='shape'):
        model.predict(np.zeros((3, len(FEATURE_NAMES))))


def test_sample_generator_writes_episode_isolated_splits(tmp_path: Path):
    model_path = Path(__file__).parents[1] / 'models' / 'diff_drive.xml'

    episode_ids = generate_dataset(
        tmp_path,
        model_path,
        seed=25012,
        episodes_per_split=1,
        duration_seconds=0.1,
        sample_every_steps=2,
    )

    episodes = load_episodes(tmp_path)
    assert len(episode_ids) == len(episodes) == 3
    assert {episode.split for episode in episodes} == {
        'train', 'validation', 'test',
    }
    assert all(len(episode.features) == 10 for episode in episodes)
