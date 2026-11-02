"""Detection metrics grouped by camera altitude and pitch."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

Box = Sequence[float]


def _valid_box(box: Box) -> bool:
    return len(box) == 4 and box[2] > 0 and box[3] > 0


def iou_xywh(first: Box, second: Box) -> float:
    """Intersection-over-union for boxes in `x, y, width, height` format."""
    if not _valid_box(first) or not _valid_box(second):
        return 0.0
    ax1, ay1, aw, ah = map(float, first)
    bx1, by1, bw, bh = map(float, second)
    ax2, ay2 = ax1 + aw, ay1 + ah
    bx2, by2 = bx1 + bw, by1 + bh
    inter_w = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    inter_h = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = inter_w * inter_h
    union = aw * ah + bw * bh - intersection
    return intersection / union if union > 0 else 0.0


def match_frame(
    ground_truth: Iterable[Dict[str, Any]],
    predictions: Iterable[Dict[str, Any]],
    iou_threshold: float = 0.5,
    score_threshold: float = 0.25,
) -> Tuple[int, int, int, int, int, int, float, float]:
    """Return counts plus localization error and IoU for matched boxes."""
    truth = [item for item in ground_truth if _valid_box(item["bbox"])]
    predicted = [
        item
        for item in predictions
        if _valid_box(item["bbox"]) and float(item.get("score", 0.0)) >= score_threshold
    ]
    predicted.sort(key=lambda item: float(item.get("score", 0.0)), reverse=True)
    used = set()
    small_total = 0
    small_hit = 0
    true_positive = 0
    localization_error_sum = 0.0
    matched_iou_sum = 0.0

    for item in truth:
        best_index = -1
        best_iou = 0.0
        for index, predicted_item in enumerate(predicted):
            if index in used:
                continue
            overlap = iou_xywh(item["bbox"], predicted_item["bbox"])
            if overlap > best_iou:
                best_iou = overlap
                best_index = index
        is_small = bool(item.get("small", False))
        small_total += int(is_small)
        if best_index >= 0 and best_iou >= iou_threshold:
            used.add(best_index)
            true_positive += 1
            small_hit += int(is_small)
            matched_iou_sum += best_iou
            truth_box = item["bbox"]
            predicted_box = predicted[best_index]["bbox"]
            truth_center_x = float(truth_box[0]) + float(truth_box[2]) / 2.0
            truth_center_y = float(truth_box[1]) + float(truth_box[3]) / 2.0
            predicted_center_x = (
                float(predicted_box[0]) + float(predicted_box[2]) / 2.0
            )
            predicted_center_y = (
                float(predicted_box[1]) + float(predicted_box[3]) / 2.0
            )
            localization_error_sum += math.hypot(
                predicted_center_x - truth_center_x,
                predicted_center_y - truth_center_y,
            )

    false_positive = len(predicted) - len(used)
    false_negative = len(truth) - true_positive
    return (
        true_positive,
        false_positive,
        false_negative,
        small_total,
        small_hit,
        len(truth),
        localization_error_sum,
        matched_iou_sum,
    )


def _safe_ratio(numerator: float, denominator: float) -> float:
    return numerator / denominator if denominator else 0.0


def evaluate_frames(
    frames: Iterable[Dict[str, Any]],
    iou_threshold: float = 0.5,
    score_threshold: float = 0.25,
) -> List[Dict[str, Any]]:
    """Aggregate precision, recall and small-object recall by altitude/pitch."""
    groups: Dict[Tuple[float, float], Dict[str, int]] = {}
    for frame in frames:
        key = (float(frame["altitude_m"]), float(frame["pitch_deg"]))
        stats = match_frame(
            frame.get("ground_truth", []),
            frame.get("predictions", []),
            iou_threshold,
            score_threshold,
        )
        group = groups.setdefault(
            key,
            {
                "tp": 0,
                "fp": 0,
                "fn": 0,
                "small_total": 0,
                "small_hit": 0,
                "gt": 0,
                "localization_error_sum": 0.0,
                "matched_iou_sum": 0.0,
            },
        )
        for name, value in zip(
            (
                "tp",
                "fp",
                "fn",
                "small_total",
                "small_hit",
                "gt",
                "localization_error_sum",
                "matched_iou_sum",
            ),
            stats,
        ):
            group[name] += value

    rows: List[Dict[str, Any]] = []
    for (altitude, pitch), group in sorted(groups.items()):
        precision = _safe_ratio(group["tp"], group["tp"] + group["fp"])
        recall = _safe_ratio(group["tp"], group["gt"])
        f1 = _safe_ratio(2 * precision * recall, precision + recall)
        small_recall = _safe_ratio(group["small_hit"], group["small_total"])
        localization_error = _safe_ratio(
            group["localization_error_sum"], group["tp"]
        )
        matched_iou = _safe_ratio(group["matched_iou_sum"], group["tp"])
        rows.append(
            {
                "altitude_m": altitude,
                "pitch_deg": pitch,
                "ground_truth": group["gt"],
                "tp": group["tp"],
                "fp": group["fp"],
                "fn": group["fn"],
                "precision": round(precision, 4),
                "recall": round(recall, 4),
                "f1": round(f1, 4),
                "small_object_recall": round(small_recall, 4),
                "localization_error_px": round(localization_error, 4),
                "matched_iou": round(matched_iou, 4),
            }
        )
    return rows


def write_csv(rows: Iterable[Dict[str, Any]], path: Path) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0].keys()) if rows else [
        "altitude_m", "pitch_deg", "ground_truth", "tp", "fp", "fn",
        "precision", "recall", "f1", "small_object_recall",
        "localization_error_px", "matched_iou",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
