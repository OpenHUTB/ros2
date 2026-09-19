# -*- coding: utf-8 -*-

import argparse
import os
import subprocess
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
os.chdir(BASE_DIR)

parser = argparse.ArgumentParser(
    description="CarlaAir YOLO perception and MLP trajectory control"
)

parser.add_argument(
    "mode",
    choices=[
        "yolo",
        "generate",
        "train",
        "square",
        "circle",
        "eight",
        "evaluate",
    ],
)

args = parser.parse_args()
python = sys.executable

if args.mode == "yolo":
    command = [python, "yolo_realtime.py"]

elif args.mode == "generate":
    command = [python, "generate_training_data.py"]

elif args.mode == "train":
    command = [python, "train_mlp.py"]

elif args.mode in ("square", "circle", "eight"):
    command = [
        python,
        "run_mlp_trajectory.py",
        "--trajectory",
        args.mode,
    ]

else:
    command = [python, "evaluate_trajectories.py"]

subprocess.check_call(command)