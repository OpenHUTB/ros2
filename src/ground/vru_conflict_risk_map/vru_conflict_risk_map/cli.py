"""CLI for the VRU conflict-risk map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .risk import evaluate_conflicts, load_ego_track, load_vru_tracks
from .swept import evaluate_swept_conflicts


def main(argv=None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego", type=Path, default=root / "data" / "ego.csv")
    parser.add_argument("--vru", type=Path, default=root / "data" / "vru.csv")
    parser.add_argument("--out", type=Path, default=Path("vru_conflict_report.json"))
    parser.add_argument("--mode", choices=("point", "swept"), default="point")
    parser.add_argument("--ego-radius", type=float, default=1.0)
    parser.add_argument("--caution-clearance", type=float, default=1.0)
    parser.add_argument("--horizon", type=float, default=5.0)
    args = parser.parse_args(argv)
    ego = load_ego_track(args.ego)
    vru = load_vru_tracks(args.vru)
    if args.mode == "swept":
        report = evaluate_swept_conflicts(
            ego,
            vru,
            ego_radius_m=args.ego_radius,
            caution_clearance_m=args.caution_clearance,
            horizon_s=args.horizon,
        )
    else:
        report = evaluate_conflicts(ego, vru)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
