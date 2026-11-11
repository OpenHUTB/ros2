"""Offline deterministic demo for environments without ROS 2."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .core import LandingConstraints, LandingSiteSelector
from .scenario import scenario_at


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--dt", type=float, default=0.5)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/landing_selector"))
    options = parser.parse_args(args)
    output_dir = options.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    selector = LandingSiteSelector(LandingConstraints())
    rows = []
    time_s = 0.0
    while time_s <= options.duration + 1e-9:
        uav, sites = scenario_at(time_s)
        decision = selector.select(uav, sites)
        rows.append(
            {
                "time_s": round(time_s, 3),
                "state": decision.state,
                "selected_site": decision.selected_site_id,
                "score": decision.score,
                "eligible_count": decision.eligible_count,
            }
        )
        time_s += options.dt
    with (output_dir / "runtime.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "samples": len(rows),
        "selected_site_counts": {
            site: sum(row["selected_site"] == site for row in rows)
            for site in sorted({row["selected_site"] for row in rows if row["selected_site"]})
        },
        "mean_score": round(
            sum(row["score"] for row in rows if row["score"] is not None)
            / max(1, sum(row["score"] is not None for row in rows)),
            3,
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
