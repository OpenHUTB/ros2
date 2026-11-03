"""Altitude and pitch robustness evaluation for aerial detections."""

from .evaluator import evaluate_frames, iou_xywh

__all__ = ["evaluate_frames", "iou_xywh"]
