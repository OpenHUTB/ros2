"""Command-line entry point for the moving-pad landing example."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_uav_moving_pad_landing.landing import (
    DEFAULT_PAD_AMPLITUDE,
    DEFAULT_PAD_FREQUENCY,
    DEFAULT_PAD_PHASE,
    MODEL_PATH,
    run_demo,
    train_controller,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo UAV visual landing on a moving platform")
    parser.add_argument("mode", choices=("demo", "train", "teleop"), nargs="?", default="demo")
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--samples", type=int, default=5000)
    parser.add_argument("--steps-per-episode", type=int, default=20)
    parser.add_argument("--pad-amplitude-x", type=float, default=DEFAULT_PAD_AMPLITUDE[0])
    parser.add_argument("--pad-amplitude-y", type=float, default=DEFAULT_PAD_AMPLITUDE[1])
    parser.add_argument("--pad-frequency-x", type=float, default=DEFAULT_PAD_FREQUENCY[0])
    parser.add_argument("--pad-frequency-y", type=float, default=DEFAULT_PAD_FREQUENCY[1])
    parser.add_argument("--pad-phase-x", type=float, default=DEFAULT_PAD_PHASE[0])
    parser.add_argument("--pad-phase-y", type=float, default=DEFAULT_PAD_PHASE[1])
    args = parser.parse_args()
    if args.mode == "train":
        result = train_controller(MODEL_PATH, samples=args.samples, steps_per_episode=args.steps_per_episode)
    elif args.mode == "teleop":
        from mujoco_uav_moving_pad_landing.teleop import run_keyboard_teleop

        result = run_keyboard_teleop(args.output)
    else:
        if not MODEL_PATH.exists():
            train_controller(MODEL_PATH, samples=args.samples, steps_per_episode=args.steps_per_episode)
        result = run_demo(
            args.output,
            steps=args.steps,
            pad_amplitude=(args.pad_amplitude_x, args.pad_amplitude_y),
            pad_frequency=(args.pad_frequency_x, args.pad_frequency_y),
            pad_phase=(args.pad_phase_x, args.pad_phase_y),
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
