"""CLI for the VRU conflict-risk map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .risk import evaluate_conflicts, load_ego_track, load_vru_tracks


def main(argv=None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego", type=Path, default=root / "data" / "ego.csv")
    parser.add_argument("--vru", type=Path, default=root / "data" / "vru.csv")
    parser.add_argument("--out", type=Path, default=Path("vru_conflict_report.json"))
    args = parser.parse_args(argv)
    report = evaluate_conflicts(load_ego_track(args.ego), load_vru_tracks(args.vru))
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
