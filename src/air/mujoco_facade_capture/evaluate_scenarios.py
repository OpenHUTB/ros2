"""对三种立面外观执行同一神经网络扫描并生成实测GIF。"""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import mujoco
from PIL import Image

from facade.learning import DenseController
from facade.simulation import run_scan
from train import neural_policy


def evaluate(metrics_path: Path, gif_dir: Path) -> dict:
    metrics_path = Path(metrics_path)
    gif_dir = Path(gif_dir)
    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    gif_dir.mkdir(parents=True, exist_ok=True)
    model = DenseController.load(
        Path(__file__).resolve().parent / "models/facade_controller.json"
    )
    report = {
        "simulator": f"MuJoCo {mujoco.__version__}",
        "controller": "neural",
        "distance_target_m": 4.0,
        "initial_offset_m": 0.5,
        "scenarios": [],
    }
    with tempfile.TemporaryDirectory(prefix="facade_scenarios_") as temporary:
        for style in ("grid", "brick", "plain"):
            result_dir = Path(temporary) / style
            metrics = run_scan(
                result_dir,
                style=style,
                distance=4.0,
                offset=0.5,
                policy=neural_policy(model, 4.0),
                capture=True,
            )
            frame_paths = [result_dir / name for name in metrics["images"]]
            frames = []
            for path in frame_paths:
                with Image.open(path) as image:
                    frames.append(image.convert("RGB"))
            gif_path = gif_dir / f"facade_scan_{style}.gif"
            if frames:
                frames[0].save(
                    gif_path,
                    save_all=True,
                    append_images=frames[1:],
                    duration=180,
                    loop=0,
                    optimize=True,
                    disposal=2,
                )
            for frame in frames:
                frame.close()
            report["scenarios"].append(
                {
                    key: metrics[key]
                    for key in (
                        "style",
                        "waypoints_total",
                        "waypoints_completed",
                        "completion_ratio",
                        "duration_s",
                        "distance_rmse_m",
                        "minimum_wall_distance_m",
                        "maximum_speed_m_s",
                    )
                }
                | {
                    "gif": gif_path.name,
                    "gif_size_bytes": gif_path.stat().st_size
                    if gif_path.exists()
                    else 0,
                }
            )
    metrics_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="执行三种立面外观的神经网络扫描评价")
    repository_root = Path(__file__).resolve().parents[3]
    parser.add_argument(
        "--metrics",
        type=Path,
        default=repository_root
        / "docs/air/mujoco_facade_capture/scenario_metrics.json",
    )
    parser.add_argument(
        "--gif-dir",
        type=Path,
        default=repository_root / "docs/img/air/mujoco_facade_capture",
    )
    args = parser.parse_args()
    print(
        json.dumps(evaluate(args.metrics, args.gif_dir), ensure_ascii=False, indent=2)
    )


if __name__ == "__main__":
    main()
