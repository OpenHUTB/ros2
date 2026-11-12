#!/usr/bin/env python3
"""Plot circular footprints, clearance, and collision TTC for the sample data."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vru_conflict_risk_map.risk import (  # noqa: E402
    load_ego_track, load_vru_tracks,
)
from vru_conflict_risk_map.swept import evaluate_swept_conflicts  # noqa: E402

COLORS = {"safe": "#2e8b57", "caution": "#d08b16", "critical": "#c62828"}
ANNOTATION_OFFSETS = {
    "p1": (-55, 28),
    "c1": (6, 32),
    "b1": (-35, 24),
}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego", type=Path, default=ROOT / "data" / "ego.csv")
    parser.add_argument("--vru", type=Path, default=ROOT / "data" / "vru.csv")
    parser.add_argument("--out", type=Path,
                        default=Path("vru_swept_conflict_demo.png"))
    args = parser.parse_args(argv)

    ego = load_ego_track(args.ego)
    vru = load_vru_tracks(args.vru)
    report = evaluate_swept_conflicts(ego, vru)
    track_meta = {track["track_id"]: track for track in report["tracks"]}

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot([p.x for p in ego], [p.y for p in ego], "o-",
            color="#20242a", lw=1.5, ms=3, label="ego trajectory")

    by_track = {}
    for point in vru:
        by_track.setdefault(point.track_id, []).append(point)
    for track_id, points in sorted(by_track.items()):
        points.sort(key=lambda point: point.timestamp)
        meta = track_meta[track_id]
        color = COLORS[meta["risk"]]
        ax.plot([p.x for p in points], [p.y for p in points], "o-",
                color=color, lw=1.2, ms=4, label=f"{track_id} ({meta['class']})")
        target = min(
            points,
            key=lambda point: abs(
                point.timestamp - meta["min_clearance_timestamp_s"]),
        )
        footprint = Circle(
            (target.x, target.y), meta["combined_radius_m"],
            facecolor=color, edgecolor=color, alpha=0.16, lw=1.2,
        )
        ax.add_patch(footprint)
        ttc = meta["collision_ttc_s"]
        ttc_text = "none" if ttc is None else f"{ttc:.2f} s"
        ax.annotate(
            f"{track_id}: {meta['risk']}\n"
            f"clearance {meta['min_clearance_m']:.2f} m\n"
            f"collision TTC {ttc_text}",
            (target.x, target.y),
            xytext=ANNOTATION_OFFSETS.get(track_id, (8, 8)),
            textcoords="offset points",
            fontsize=8,
            color=color,
            ha="left",
            va="bottom",
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1.0, 24.0)
    ax.set_ylim(-2.0, 2.0)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("VRU swept-footprint conflict analysis")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    plt.close(fig)
    print(f"written: {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
