"""CSV export of the per-sample separation series (standard library only)."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Iterable, Mapping

FIELDS = ("time_s", "uav_a", "uav_b", "separation_m")


def write_separation_csv(
    rows: Iterable[Mapping[str, Any]], path: Path | str
) -> Path:
    """Write separation rows as CSV and return the path that was written.

    The header is always written, even when there is only one UAV and therefore
    no pair to evaluate, so downstream scripts can rely on the column names.
    """
    target = Path(path)
    if target.parent and not target.parent.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in FIELDS})
    return target
