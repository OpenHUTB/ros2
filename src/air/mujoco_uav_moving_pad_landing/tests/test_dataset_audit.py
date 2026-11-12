import json

import numpy as np
import pytest

from mujoco_uav_moving_pad_landing.dataset_audit import audit_grouped_dataset, write_audit_report


def test_audit_reports_shapes_ranges_and_disjoint_episodes(tmp_path):
    features = np.arange(48, dtype=float).reshape(8, 6)
    labels = np.arange(24, dtype=float).reshape(8, 3)
    episode_ids = np.repeat(np.arange(4), 2)

    report = audit_grouped_dataset(features, labels, episode_ids, test_size=0.5, seed=5)
    target = tmp_path / "reports" / "dataset-audit.json"
    write_audit_report(report, target)

    saved = json.loads(target.read_text(encoding="utf-8"))
    assert saved["sample_count"] == 8
    assert saved["episode_count"] == 4
    assert saved["train_sample_count"] + saved["test_sample_count"] == 8
    assert saved["train_episode_count"] + saved["test_episode_count"] == 4
    assert saved["episode_overlap"] == []
    assert saved["feature_min"] == features.min(axis=0).tolist()
    assert saved["label_max"] == labels.max(axis=0).tolist()


def test_audit_rejects_non_finite_samples():
    features = np.zeros((4, 2), dtype=float)
    features[0, 0] = np.nan
    labels = np.zeros((4, 1), dtype=float)
    episode_ids = np.array([0, 0, 1, 1])

    with pytest.raises(ValueError, match="finite"):
        audit_grouped_dataset(features, labels, episode_ids)
