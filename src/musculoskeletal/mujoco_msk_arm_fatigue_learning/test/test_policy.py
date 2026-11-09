import json
from pathlib import Path

import pytest

from mujoco_msk_arm_fatigue_learning.policy import LearningPolicy


def test_policy_round_trip():
    path = Path(__file__).with_name('_policy_round_trip.json')
    try:
        policy = LearningPolicy(
            learning_rate=42.0,
            gain_decay_rate=0.1,
            maximum_gain=1.7,
        )
        policy.save(path)
        assert LearningPolicy.load(path) == policy
    finally:
        path.unlink(missing_ok=True)


def test_unknown_policy_key_is_rejected():
    path = Path(__file__).with_name('_policy_unknown_key.json')
    try:
        path.write_text(
            json.dumps({'learning_rate': 20.0, 'unknown': 1.0}),
            encoding='utf-8',
        )
        with pytest.raises(ValueError, match='unknown'):
            LearningPolicy.load(path)
    finally:
        path.unlink(missing_ok=True)
