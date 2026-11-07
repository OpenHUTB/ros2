"""Command-line entry point for the moving-pad landing example."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_uav_moving_pad_landing.landing import MODEL_PATH, run_demo, train_controller


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo UAV visual landing on a moving platform")
    parser.add_argument("mode", choices=("demo", "train", "teleop", "navigate"), nargs="?", default="demo")
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--samples", type=int, default=5000)
    args = parser.parse_args()
    if args.mode == "train":
        result = train_controller(MODEL_PATH, samples=args.samples)
    elif args.mode == "teleop":
        from mujoco_uav_moving_pad_landing.teleop import run_keyboard_teleop

        result = run_keyboard_teleop(args.output)
    elif args.mode == "navigate":
        from mujoco_uav_moving_pad_landing.navigation import run_slam_navigation

        result = run_slam_navigation(args.output)
    else:
        if not MODEL_PATH.exists():
            train_controller(MODEL_PATH, samples=args.samples)
        result = run_demo(args.output, steps=args.steps)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
