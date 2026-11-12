"""检查分组着陆数据集在训练与指标报告前的完整性。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .dataset import episode_split_indices


def audit_grouped_dataset(
    features: np.ndarray,
    labels: np.ndarray,
    episode_ids: np.ndarray,
    *,
    test_size: float = 0.2,
    seed: int = 42,
) -> dict[str, Any]:
    """汇总数据形状、有限值范围及 episode 拆分完整性。"""
    features = np.asarray(features, dtype=np.float64)
    labels = np.asarray(labels, dtype=np.float64)
    episode_ids = np.asarray(episode_ids)

    if features.ndim != 2 or labels.ndim != 2 or episode_ids.ndim != 1:
        raise ValueError("features and labels must be matrices and episode_ids a vector")
    if not len(features) or len(features) != len(labels) or len(features) != len(episode_ids):
        raise ValueError("features, labels, and episode_ids must have equal non-zero rows")
    if not np.isfinite(features).all() or not np.isfinite(labels).all():
        raise ValueError("features and labels must contain only finite values")

    train_rows, test_rows = episode_split_indices(episode_ids, test_size=test_size, seed=seed)
    train_episodes = set(episode_ids[train_rows].tolist())
    test_episodes = set(episode_ids[test_rows].tolist())
    overlap = train_episodes.intersection(test_episodes)
    if overlap:
        raise RuntimeError("episode leakage detected in dataset split")

    return {
        "sample_count": int(len(features)),
        "feature_count": int(features.shape[1]),
        "label_count": int(labels.shape[1]),
        "episode_count": int(len(np.unique(episode_ids))),
        "train_sample_count": int(len(train_rows)),
        "test_sample_count": int(len(test_rows)),
        "train_episode_count": int(len(train_episodes)),
        "test_episode_count": int(len(test_episodes)),
        "episode_overlap": [],
        "feature_min": features.min(axis=0).tolist(),
        "feature_max": features.max(axis=0).tolist(),
        "label_min": labels.min(axis=0).tolist(),
        "label_max": labels.max(axis=0).tolist(),
    }


def write_audit_report(report: dict[str, Any], path: Path) -> None:
    """将审计摘要以 UTF-8 JSON 格式写入文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
