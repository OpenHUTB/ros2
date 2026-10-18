"""CLI wrapper for VRU twin metrics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .metrics import evaluate_vru_tracks, load_vru_tracks


def main(argv=None) -> int:
    module_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gt", type=Path, default=module_root / "data" / "vru_gt.csv"
    )
    parser.add_argument(
        "--pred", type=Path, default=module_root / "data" / "vru_pred.csv"
    )
    parser.add_argument("--max-distance", type=float, default=1.5)
    parser.add_argument("--out", type=Path, default=Path("vru_metrics.json"))
    args = parser.parse_args(argv)

    metrics = evaluate_vru_tracks(
        load_vru_tracks(args.gt),
        load_vru_tracks(args.pred),
        max_distance=args.max_distance,
    )
    payload = metrics.as_dict()
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
