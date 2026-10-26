"""CLI wrapper for the driving-study analyzer."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .analyzer import analyze_files


def main(argv=None) -> int:
    module_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--driving",
        type=Path,
        default=module_root / "data" / "sample_session.csv",
    )
    parser.add_argument(
        "--gaze",
        type=Path,
        default=module_root / "data" / "sample_session_gaze.csv",
    )
    parser.add_argument("--out", type=Path, default=Path("driving_study_report.json"))
    args = parser.parse_args(argv)

    report = analyze_files(args.driving, args.gaze)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
