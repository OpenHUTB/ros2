"""Training loops (NumPy-only) for the four neural modules."""

from .train import (train_control, train_e2e, train_perception, train_planner)

__all__ = ["train_perception", "train_planner", "train_control", "train_e2e"]
