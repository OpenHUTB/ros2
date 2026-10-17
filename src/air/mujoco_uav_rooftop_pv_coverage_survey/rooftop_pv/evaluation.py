"""Scenario evaluation against deterministic baselines."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import numpy as np
from PIL import Image

from .training import _records, segmentation_metrics


def evaluate_scenarios(dataset: Path) -> dict[str, Any]:
    rows = [json.loads(line) for line in (dataset / "manifest.jsonl").read_text(encoding="utf-8").splitlines()]
    result: dict[str, Any] = {}
    for scenario in sorted({row["scenario"] for row in rows}):
        scenario_rows = [row for row in rows if row["scenario"] == scenario]
        metrics = []
        for row in scenario_rows:
            truth = np.asarray(Image.open(dataset / row["mask_path"]).convert("L")) > 0
            rgb = np.asarray(Image.open(dataset / row["rgb_path"]).convert("RGB"))
            rule = (rgb[:, :, 0] > 100) & (rgb[:, :, 1] > 100) & (rgb[:, :, 2] < 180)
            metrics.append(segmentation_metrics(__import__("torch").from_numpy(rule[None, None]), __import__("torch").from_numpy(truth[None, None])))
        result[scenario] = {key: float(np.mean([item[key] for item in metrics])) for key in metrics[0]}
        result[scenario]["frames"] = len(scenario_rows)
    return result
