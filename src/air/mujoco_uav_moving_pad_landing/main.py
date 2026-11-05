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
    parser.add_argument(
        "mode",
        choices=("demo", "train", "train-image", "train-navigation", "teleop", "navigate"),
        nargs="?",
        default="demo",
    )
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--samples", type=int, default=5000)
    parser.add_argument("--controller", choices=("features", "image"), default="features")
    args = parser.parse_args()
    if args.mode == "train":
        result = train_controller(MODEL_PATH, samples=args.samples)
    elif args.mode == "train-image":
        from mujoco_uav_moving_pad_landing.image_controller import train_image_controller

        result = train_image_controller(samples=args.samples)
    elif args.mode == "train-navigation":
        from mujoco_uav_moving_pad_landing.navigation_policy import train_navigation_policy

        result = train_navigation_policy()
    elif args.mode == "teleop":
        from mujoco_uav_moving_pad_landing.teleop import run_keyboard_teleop

        result = run_keyboard_teleop(args.output)
    elif args.mode == "navigate":
        from mujoco_uav_moving_pad_landing.navigation_policy import run_navigation_suite

        result = run_navigation_suite(args.output)
    else:
        if args.controller == "image":
            from mujoco_uav_moving_pad_landing.image_controller import IMAGE_MODEL_PATH, train_image_controller

            if not IMAGE_MODEL_PATH.exists():
                train_image_controller(samples=args.samples)
            result = run_demo(args.output, steps=args.steps, end_to_end=True)
        else:
            if not MODEL_PATH.exists():
                train_controller(MODEL_PATH, samples=args.samples)
            result = run_demo(args.output, steps=args.steps)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
