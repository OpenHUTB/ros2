"""Dataset integrity and leakage checks / 数据集完整性与泄漏检查。"""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


REQUIRED_FIELDS = {
    "frame_id",
    "scenario",
    "sequence_id",
    "frame_index",
    "split",
    "rgb_path",
    "mask_path",
    "pose",
    "expert_input",
    "expert_command",
    "pv_pixels",
    "total_pixels",
    "pv_pixel_ratio",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65_536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(path: Path) -> list[dict[str, Any]]:
    """Load non-empty JSON Lines records / 读取非空 JSON Lines 记录。"""
    if not path.is_file():
        raise FileNotFoundError(f"manifest is missing / manifest 缺失: {path}")
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not records:
        raise ValueError("manifest is empty / manifest 为空")
    return records


def audit_dataset(root: Path) -> dict[str, Any]:
    """Check files, dimensions, labels, duplicates, and split isolation / 检查文件、尺寸、标签、重复项与划分隔离。"""
    records = load_manifest(root / "manifest.jsonl")
    missing_fields: list[str] = []
    missing_files: list[str] = []
    invalid_masks: list[str] = []
    dimension_mismatches: list[str] = []
    frame_ids: set[str] = set()
    duplicate_frame_ids: list[str] = []
    sequence_splits: dict[str, set[str]] = defaultdict(set)
    content_keys: dict[tuple[str, str], list[str]] = defaultdict(list)
    ratios_by_scenario: dict[str, list[float]] = defaultdict(list)

    for record in records:
        frame_id = str(record.get("frame_id", ""))
        absent = REQUIRED_FIELDS - set(record)
        if absent:
            missing_fields.append(f"{frame_id}:{','.join(sorted(absent))}")
            continue
        if frame_id in frame_ids:
            duplicate_frame_ids.append(frame_id)
        frame_ids.add(frame_id)
        sequence_splits[str(record["sequence_id"])].add(str(record["split"]))
        rgb_path = root / str(record["rgb_path"])
        mask_path = root / str(record["mask_path"])
        if not rgb_path.is_file() or not mask_path.is_file():
            missing_files.append(frame_id)
            continue
        with Image.open(rgb_path) as rgb_image, Image.open(mask_path) as mask_image:
            if rgb_image.size != mask_image.size:
                dimension_mismatches.append(frame_id)
            mask_array = np.asarray(mask_image)
            if not set(np.unique(mask_array).tolist()).issubset({0, 255}):
                invalid_masks.append(frame_id)
            actual_pixels = int(np.count_nonzero(mask_array))
            if actual_pixels != int(record["pv_pixels"]) or mask_array.size != int(record["total_pixels"]):
                invalid_masks.append(frame_id)
        content_keys[(_sha256(rgb_path), _sha256(mask_path))].append(frame_id)
        ratios_by_scenario[str(record["scenario"])].append(float(record["pv_pixel_ratio"]))

    split_leakage = sorted(sequence for sequence, splits in sequence_splits.items() if len(splits) != 1)
    duplicate_content = sorted(group for group in content_keys.values() if len(group) > 1)
    errors = {
        "missing_fields": sorted(missing_fields),
        "missing_files": sorted(missing_files),
        "duplicate_frame_ids": sorted(duplicate_frame_ids),
        "invalid_masks": sorted(set(invalid_masks)),
        "dimension_mismatches": sorted(dimension_mismatches),
        "sequence_split_leakage": split_leakage,
        "duplicate_content": duplicate_content,
    }
    report = {
        "schema_version": 1,
        "status": "pass" if not any(errors.values()) else "fail",
        "frame_count": len(records),
        "sequence_count": len(sequence_splits),
        "split_frame_counts": {
            split: sum(record.get("split") == split for record in records)
            for split in ("train", "validation", "test")
        },
        "pv_pixel_ratio_by_scenario": {
            scenario: {
                "minimum": min(values),
                "mean": sum(values) / len(values),
                "maximum": max(values),
            }
            for scenario, values in sorted(ratios_by_scenario.items())
        },
        "errors": errors,
    }
    report_path = root / "quality_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if report["status"] != "pass":
        raise ValueError(f"dataset quality audit failed / 数据质量检查失败: {report_path}")
    return report
