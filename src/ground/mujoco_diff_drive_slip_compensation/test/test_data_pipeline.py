"""Tests for deterministic multi-friction episode collection."""

import csv
from dataclasses import replace
import json
from pathlib import Path

import pytest

from mujoco_diff_drive_slip_compensation.command_profile import ExcitationProfile
from mujoco_diff_drive_slip_compensation.dataset import (
    DatasetSample,
    EpisodeCsvWriter,
    assign_episode_split,
)
from mujoco_diff_drive_slip_compensation.scenarios import FrictionSchedule
from mujoco_diff_drive_slip_compensation.sensors import SensorNoise, SensorNoiseConfig


def _sample(episode_id: str, split: str) -> DatasetSample:
    values = {
        name: 0.0
        for name in DatasetSample.__dataclass_fields__
        if name not in {'timestamp_ns', 'episode_id', 'split'}
    }
    values['friction_coefficient'] = 0.9
    return DatasetSample(
        timestamp_ns=123,
        episode_id=episode_id,
        split=split,
        **values,
    )


def test_friction_schedule_tracks_complete_episodes():
    schedule = FrictionSchedule([0.25, 0.55, 0.9], 30.0)

    first = schedule.at(29.999)
    second = schedule.at(30.0)
    fourth = schedule.at(90.0)

    assert first.episode_number == 0
    assert first.episode_id == 'episode_0000_mu_0p250'
    assert second.coefficient == pytest.approx(0.55)
    assert second.episode_number == 1
    assert fourth.index == 0
    assert fourth.episode_number == 3


@pytest.mark.parametrize('time_value', [-0.1, float('nan'), float('inf')])
def test_friction_schedule_rejects_invalid_time(time_value):
    with pytest.raises(ValueError):
        FrictionSchedule([0.5], 10.0).at(time_value)


def test_sensor_noise_is_reproducible_for_equal_seeds():
    first = SensorNoise(SensorNoiseConfig(), seed=25012)
    second = SensorNoise(SensorNoiseConfig(), seed=25012)

    assert first.encoders([0.0] * 4, [1.0] * 4) == second.encoders(
        [0.0] * 4, [1.0] * 4
    )
    assert first.imu([0.0, 0.0, 9.81], [0.0] * 3) == second.imu(
        [0.0, 0.0, 9.81], [0.0] * 3
    )


def test_excitation_profile_covers_longitudinal_and_yaw_motion():
    profile = ExcitationProfile(period_seconds=12.0)

    commands = [profile.command_at(value) for value in (0.0, 3.0, 6.0, 9.0)]

    assert [command.linear for command in commands] == [0.6, 0.4, -0.3, 0.4]
    assert [command.angular for command in commands] == [0.0, 0.8, 0.0, -0.8]


def test_episode_split_is_stable_and_episode_scoped():
    episode_id = 'episode_0007_mu_0p550'

    assignments = {assign_episode_split(episode_id, 25012) for _ in range(20)}

    assert len(assignments) == 1
    assert assignments <= {'train', 'validation', 'test'}


def test_episode_writer_creates_csv_and_version_manifest(tmp_path: Path):
    episode_id = 'episode_0002_mu_0p900'
    split = assign_episode_split(episode_id, 25012)
    writer = EpisodeCsvWriter(
        tmp_path,
        episode_id,
        split_seed=25012,
        metadata={'mujoco_version': '3.12.0', 'random_seed': 25012},
        flush_interval=1,
    )
    writer.write(_sample(episode_id, split))
    writer.close()

    with writer.csv_path.open(encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))
    manifest = json.loads(writer.manifest_path.read_text(encoding='utf-8'))

    assert len(rows) == 1
    assert rows[0]['episode_id'] == episode_id
    assert rows[0]['split'] == split
    assert manifest['schema_version'] == '1.0'
    assert manifest['rows'] == 1
    assert manifest['mujoco_version'] == '3.12.0'


@pytest.mark.parametrize('episode_id', ['../escape', 'contains space', ''])
def test_episode_writer_rejects_unsafe_identifier(tmp_path: Path, episode_id):
    with pytest.raises(ValueError):
        EpisodeCsvWriter(tmp_path, episode_id, 25012, metadata={})


def test_episode_writer_rejects_non_finite_sample(tmp_path: Path):
    episode_id = 'episode_0003_mu_0p550'
    writer = EpisodeCsvWriter(tmp_path, episode_id, 25012, metadata={})
    sample = _sample(episode_id, writer.split)

    with pytest.raises(ValueError, match='finite'):
        writer.write(replace(sample, imu_acceleration_x=float('nan')))
    writer.close()
