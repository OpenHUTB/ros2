"""Entry point for training / export / eval / gif recording.

Usage:
  python main.py --train-ppo            # train PPO avoidance policy
  python main.py --eval                 # reactive avoidance rollout
  python main.py --gif                  # render demo gif
"""
from __future__ import annotations
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PKG_ROOT))


def main():
    print("humanoid_lidar_nn_perception entry point.")
    print("Use scripts/train_perception_cnn.py, scripts/train_avoid_ppo.py, "
          "scripts/record_gif.py directly.")


if __name__ == "__main__":
    main()
