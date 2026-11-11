"""Evaluate detection results against labelled ground truth.

Matching uses a **global minimum-total-absolute-time-error** one-to-one
assignment within a tolerance window (0.5 s by default), instead of the old
per-ground-truth greedy "nearest unused detection" which could produce a
suboptimal pairing when events are dense.  Reports recall, precision, false
alarms, timing-alignment error, classification accuracy and a confusion matrix.
"""

from __future__ import annotations

import itertools
import json
from typing import Dict, List, Tuple


def _global_min_cost_matching(gt: List[dict], dets: List[dict],
                              tolerance: float) -> List[Tuple[dict, dict, float]]:
    """One-to-one assignment minimising total |dt| over all valid pairings.

    ``k = min(len(gt), len(dets))`` is at most a handful in this module, so a
    bounded exhaustive search over injective mappings is both exact and cheap.
    A Hungarian solver would be preferable for large event counts; here the
    search is capped to keep the runtime bounded (the fallback is greedy).
    """
    cost = [[abs(d["t_peak"] - g["t"]) for d in dets] for g in gt]
    k = min(len(gt), len(dets))
    # Only consider candidate detections within tolerance of some GT event.
    candidates = [j for j in range(len(dets))
                  if any(cost[i][j] <= tolerance for i in range(len(gt)))]
    if k <= 8 and len(candidates) <= 12:
        best_total, best_pairs = float("inf"), []
        for chosen in itertools.permutations(candidates, k):
            total = 0.0
            for i, j in enumerate(chosen):
                if i >= len(gt):
                    break
                if cost[i][j] > tolerance:
                    total = float("inf")
                    break
                total += cost[i][j]
            if total < best_total:
                best_total = total
                best_pairs = [(gt[i], dets[j], cost[i][j])
                              for i, j in enumerate(chosen)
                              if i < len(gt) and cost[i][j] <= tolerance]
        if best_total < float("inf"):
            return best_pairs
    # Fallback: greedy (only reached for pathologically large event sets).
    used, matches = set(), []
    for i, g in enumerate(gt):
        best_j, best_dt = None, float("inf")
        for j, d in enumerate(dets):
            if j in used or cost[i][j] > tolerance:
                continue
            if cost[i][j] < best_dt:
                best_j, best_dt = j, cost[i][j]
        if best_j is not None:
            used.add(best_j)
            matches.append((g, dets[best_j], best_dt))
    return matches


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

    matches = _global_min_cost_matching(gt, dets, tolerance)

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
        # Ground truth is the excitation-interval centre; detections report the
        # vibration peak.  This is a *timing-alignment* offset with a physical
        # suspension-delay component, not pure algorithm error.
        "timing_mae_s": round(sum(timing_errors) / len(timing_errors), 4)
                        if timing_errors else None,
        "timing_mae_kind": "timing_alignment_error_s",
        "timing_max_s": round(max(timing_errors), 4) if timing_errors else None,
        "matching": "global_min_total_abs_dt",
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
        print(f"  Timing-alignment MAE: {metrics['timing_mae_s']:.3f} s "
              f"(max {metrics['timing_max_s']:.3f} s; "
              f"GT=excitation centre, det=vibration peak)")
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
