"""Evaluate detection results against labelled ground truth.

Matching uses a one-to-one nearest-time assignment within a tolerance window
(0.5 s by default).  Reports recall, precision, false alarms, timing error,
classification accuracy and a confusion matrix.
"""

from __future__ import annotations

import json
from typing import Dict, List


def evaluate(data_path: str, results_path: str,
             tolerance: float = 0.5) -> Dict:
    """Compare detections with ground truth events.

    Returns a dict with detection and classification metrics.
    """
    with open(data_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    with open(results_path, "r", encoding="utf-8") as f:
        results = json.load(f)

    gt = data.get("ground_truth", [])
    dets = results.get("events", [])
    if not gt:
        return {"error": "no ground truth in dataset"}

    # one-to-one nearest-time matching
    used = set()
    matches = []          # (gt, det)
    for g in gt:
        best_i, best_dt = None, float("inf")
        for i, d in enumerate(dets):
            if i in used:
                continue
            dt = abs(d["t_peak"] - g["t"])
            if dt < best_dt and dt <= tolerance:
                best_i, best_dt = i, dt
        if best_i is not None:
            used.add(best_i)
            matches.append((g, dets[best_i], best_dt))

    tp = len(matches)
    fn = len(gt) - tp
    fp = len(dets) - tp
    timing_errors = [m[2] for m in matches]

    labels = ["pothole", "speed_bump", "rough_patch"]
    confusion = {g: {p: 0 for p in labels} for g in labels}
    correct = 0
    for g, d, _ in matches:
        confusion[g["type"]][d["label"]] += 1
        if g["type"] == d["label"]:
            correct += 1

    return {
        "n_ground_truth": len(gt),
        "n_detections": len(dets),
        "true_positives": tp,
        "false_negatives": fn,
        "false_positives": fp,
        "recall": round(tp / len(gt), 4) if gt else 0.0,
        "precision": round(tp / len(dets), 4) if dets else 0.0,
        "timing_mae_s": round(sum(timing_errors) / len(timing_errors), 4)
                        if timing_errors else None,
        "timing_max_s": round(max(timing_errors), 4) if timing_errors else None,
        "classification_correct": correct,
        "classification_accuracy": round(correct / tp, 4) if tp else 0.0,
        "confusion_matrix": confusion,
        "tolerance_s": tolerance,
        "matches": [
            {"gt_t": g["t"], "gt_type": g["type"],
             "det_t": d["t_peak"], "det_label": d["label"],
             "dt_s": round(dt, 4), "peak_az": d["peak_az"]}
            for g, d, dt in matches
        ],
        "missed_gt": [g["t"] for g in gt
                      if g not in [m[0] for m in matches]],
    }


def print_report(metrics: Dict) -> None:
    """Pretty-print the evaluation report."""
    if "error" in metrics:
        print(f"  {metrics['error']}")
        return
    print("\n  ── Ground-truth evaluation ──")
    print(f"  GT events: {metrics['n_ground_truth']}   "
          f"Detections: {metrics['n_detections']}")
    print(f"  TP={metrics['true_positives']}  "
          f"FN={metrics['false_negatives']}  FP={metrics['false_positives']}")
    print(f"  Recall:    {metrics['recall'] * 100:.1f}%")
    print(f"  Precision: {metrics['precision'] * 100:.1f}%")
    if metrics["timing_mae_s"] is not None:
        print(f"  Timing MAE: {metrics['timing_mae_s']:.3f} s "
              f"(max {metrics['timing_max_s']:.3f} s)")
    print(f"  Type accuracy: {metrics['classification_correct']}/"
          f"{metrics['true_positives']} "
          f"({metrics['classification_accuracy'] * 100:.1f}%)")
    print("  Confusion (rows=truth, cols=predicted):")
    labels = ["pothole", "speed_bump", "rough_patch"]
    print(f"    {'':13s}" + "".join(f"{l[:10]:>12s}" for l in labels))
    for g in labels:
        row = "".join(f"{metrics['confusion_matrix'][g][p]:12d}" for p in labels)
        print(f"    {g:13s}{row}")


if __name__ == "__main__":
    import sys
    dp = sys.argv[1] if len(sys.argv) > 1 else "data/imu_drive.json"
    rp = sys.argv[2] if len(sys.argv) > 2 else "data/detection_results.json"
    print_report(evaluate(dp, rp))
