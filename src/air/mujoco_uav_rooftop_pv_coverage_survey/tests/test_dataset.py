"""Dataset generation and quality tests / 数据生成与质量测试。"""

from pathlib import Path

import numpy as np
from PIL import Image

from rooftop_pv.dataset import DatasetConfig, assign_splits, generate_dataset
from rooftop_pv.quality import audit_dataset, load_manifest
from rooftop_pv.route import RouteConfig


def test_split_assignment_is_reproducible_and_sequence_isolated() -> None:
    sequence_ids = ["sparse_000", "dense_000", "occluded_000", "sparse_001"]
    first = assign_splits(sequence_ids, 42)
    second = assign_splits(reversed(sequence_ids), 42)
    assert first == second
    assert set(first.values()) == {"train", "validation", "test"}
    assert set(first) == set(sequence_ids)


def test_end_to_end_dataset_has_aligned_rgb_masks_and_complete_manifest(tmp_path: Path) -> None:
    summary = generate_dataset(
        tmp_path,
        RouteConfig(),
        DatasetConfig(frames_per_scene=3, image_width=64, image_height=48, seed=17),
    )
    records = load_manifest(tmp_path / "manifest.jsonl")
    assert summary["frame_count"] == 9
    assert len(records) == 9
    assert len({record["frame_id"] for record in records}) == len(records)
    assert all(record["split"] in {"train", "validation", "test"} for record in records)
    for record in records:
        rgb_path = tmp_path / record["rgb_path"]
        mask_path = tmp_path / record["mask_path"]
        with Image.open(rgb_path) as rgb, Image.open(mask_path) as mask:
            assert rgb.size == mask.size == (64, 48)
            mask_array = np.asarray(mask)
            assert set(np.unique(mask_array).tolist()).issubset({0, 255})
            assert int(np.count_nonzero(mask_array)) == record["pv_pixels"]
    report = audit_dataset(tmp_path)
    assert report["status"] == "pass"
    assert not report["errors"]["sequence_split_leakage"]
    assert not report["errors"]["duplicate_content"]


def test_mask_positive_pixels_correspond_to_blue_panels(tmp_path: Path) -> None:
    generate_dataset(
        tmp_path,
        RouteConfig(),
        DatasetConfig(frames_per_scene=3, image_width=96, image_height=72, seed=23),
        scenarios=("sparse", "dense", "occluded"),
    )
    records = load_manifest(tmp_path / "manifest.jsonl")
    checked = 0
    for record in records:
        rgb = np.asarray(Image.open(tmp_path / record["rgb_path"]).convert("RGB"))
        mask = np.asarray(Image.open(tmp_path / record["mask_path"]).convert("L")) > 0
        if not np.any(mask):
            continue
        panel_pixels = rgb[mask]
        assert float(panel_pixels[:, 2].mean()) > float(panel_pixels[:, 0].mean())
        checked += 1
    assert checked > 0
