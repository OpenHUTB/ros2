"""Evaluate detection robustness over camera altitude and pitch."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import List

try:
    from .evaluator import evaluate_frames, write_csv
except ImportError:
    from evaluator import evaluate_frames, write_csv


DEFAULT_INPUT = Path(__file__).resolve().parent / "sample_data" / "sample_frames.json"


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate detector quality grouped by UAV altitude and pitch"
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/altitude_pitch_metrics.csv"),
    )
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--score-threshold", type=float, default=0.25)
    parser.add_argument("--frame-delay", type=float, default=0.0)
    args = parser.parse_args(argv)

    try:
        payload = json.loads(args.input.read_text(encoding="utf-8"))
        rows = evaluate_frames(
            payload["frames"],
            iou_threshold=args.iou_threshold,
            score_threshold=args.score_threshold,
        )
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    if not rows:
        print("ERROR: no frames were evaluated")
        return 1

    print("altitude  pitch  gt  tp  fp  fn  precision  recall    f1  small_recall")
    for row in rows:
        print(
            f"{row['altitude_m']:>8.1f} {row['pitch_deg']:>6.1f} "
            f"{row['ground_truth']:>3} {row['tp']:>3} {row['fp']:>3} "
            f"{row['fn']:>3} {row['precision']:>9.3f} {row['recall']:>7.3f} "
            f"{row['f1']:>5.3f} {row['small_object_recall']:>12.3f}"
        )
        if args.frame_delay > 0:
            time.sleep(args.frame_delay)

    write_csv(rows, args.output)
    print(f"CSV: {args.output}")
    print(f"PASS: evaluated {len(rows)} altitude/pitch groups")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
