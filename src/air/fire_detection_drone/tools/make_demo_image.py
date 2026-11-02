"""Generate a deterministic synthetic detection preview for the README/docs."""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_ROOT)

from fire_detection_drone.detector_core import (
    HsvFireDetector,
    annotate,
)


def build_scene(width: int = 960, height: int = 640) -> np.ndarray:
    scene = np.zeros((height, width, 3), dtype=np.uint8)
    scene[:, :] = (48, 44, 40)

    # Horizon and ground.
    cv2.rectangle(scene, (0, int(height * 0.55)), (width, height), (70, 74, 76), -1)
    cv2.line(
        scene,
        (0, int(height * 0.55)),
        (width, int(height * 0.55)),
        (105, 108, 110),
        2,
    )

    # Buildings / non-fire distractors.
    cv2.rectangle(scene, (80, 210), (260, 430), (95, 88, 80), -1)
    cv2.rectangle(scene, (300, 260), (430, 430), (82, 92, 98), -1)
    cv2.rectangle(scene, (660, 240), (820, 430), (88, 86, 84), -1)
    for x in range(100, 250, 40):
        cv2.rectangle(scene, (x, 235), (x + 20, 270), (150, 160, 165), -1)
    cv2.rectangle(scene, (690, 270), (790, 360), (120, 130, 135), -1)

    # Flame near a building at the right side.
    cx, base = 560, 430
    outer = np.array([
        [cx, base],
        [cx - 55, base - 45],
        [cx - 38, base - 95],
        [cx - 48, base - 135],
        [cx - 10, base - 205],
        [cx + 20, base - 145],
        [cx + 12, base - 100],
        [cx + 48, base - 65],
        [cx + 55, base - 20],
    ], dtype=np.int32)
    middle = np.array([
        [cx, base - 5],
        [cx - 34, base - 48],
        [cx - 22, base - 95],
        [cx - 2, base - 160],
        [cx + 18, base - 100],
        [cx + 30, base - 55],
    ], dtype=np.int32)
    core = np.array([
        [cx, base - 10],
        [cx - 16, base - 50],
        [cx - 4, base - 110],
        [cx + 12, base - 55],
    ], dtype=np.int32)

    glow = np.zeros_like(scene)
    cv2.fillPoly(glow, [outer], (0, 90, 255))
    glow = cv2.GaussianBlur(glow, (0, 0), 18)
    scene = cv2.addWeighted(scene, 1.0, glow, 0.65, 0)

    cv2.fillPoly(scene, [outer], (0, 120, 255))
    cv2.fillPoly(scene, [middle], (0, 205, 255))
    cv2.fillPoly(scene, [core], (235, 255, 255))

    # Small embers.
    for offset_x, offset_y, radius in ((80, -180, 4), (-70, -150, 3), (35, -230, 3)):
        cv2.circle(
            scene,
            (cx + offset_x, base + offset_y),
            radius,
            (0, 180, 255),
            -1,
        )

    cv2.putText(
        scene,
        'SYNTHETIC TEST SCENE - NOT AIRSIM',
        (16, height - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (210, 210, 210),
        2,
        cv2.LINE_AA,
    )
    return scene


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        default=os.path.join(
            'docs', 'img', 'air', 'fire_detection_drone', 'demo_detection.png'
        ),
    )
    args = parser.parse_args()

    scene = build_scene()
    detector = HsvFireDetector()
    detections = detector.detect(scene)
    if not detections:
        raise SystemExit('Synthetic flame was not detected')

    preview = annotate(scene, detections, alert=True)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    if not cv2.imwrite(args.output, preview):
        raise SystemExit(f'Failed to write {args.output}')

    best = detections[0]
    print(
        f'wrote {args.output}: detections={len(detections)}, '
        f'confidence={best.confidence:.3f}, bbox={best.bbox}'
    )


if __name__ == '__main__':
    main()
