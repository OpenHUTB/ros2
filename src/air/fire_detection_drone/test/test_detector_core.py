import cv2
import numpy as np
from fire_detection_drone.detector_core import (
    AlertTracker,
    FireDetectorConfig,
    HsvFireDetector,
    OnnxFireVerifier,
    annotate,
    apply_verifier_confidence,
)


def make_fire_frame(width: int = 640, height: int = 480) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    points = np.array([
        [420, 420],
        [355, 340],
        [370, 260],
        [345, 210],
        [420, 90],
        [465, 220],
        [450, 300],
        [490, 365],
    ], dtype=np.int32)
    cv2.fillPoly(frame, [points], (0, 120, 255))
    cv2.ellipse(frame, (420, 330), (32, 70), 0, 0, 360, (0, 220, 255), -1)
    cv2.ellipse(frame, (420, 350), (12, 34), 0, 0, 360, (235, 255, 255), -1)
    return frame


def test_detects_synthetic_flame():
    detector = HsvFireDetector()
    detections = detector.detect(make_fire_frame())
    assert detections
    best = detections[0]
    assert 0.0 < best.confidence <= 1.0
    assert best.x_min < 420 < best.x_max
    assert best.y_min < 350 < best.y_max
    assert 0.0 < best.center_x < 1.0
    assert 0.0 < best.center_y < 1.0


def test_ignores_blue_scene():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100, 100), (500, 400), (255, 0, 0), -1)
    assert HsvFireDetector().detect(frame) == []


def test_ignores_tiny_colour_noise():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (100, 100), (104, 104), (0, 120, 255), -1)
    assert HsvFireDetector().detect(frame) == []


def test_annotate_returns_new_image():
    frame = make_fire_frame()
    detections = HsvFireDetector().detect(frame)
    preview = annotate(frame, detections, alert=True)
    assert preview.shape == frame.shape
    assert preview.dtype == np.uint8
    assert not np.array_equal(preview, frame)


def test_detector_handles_empty_input():
    detector = HsvFireDetector()
    assert detector.detect(None) == []
    assert detector.detect(np.zeros((0, 0, 3), dtype=np.uint8)) == []


def test_verifier_confidence_blend():
    detector = HsvFireDetector(
        FireDetectorConfig(min_confidence=0.0)
    )
    detection = detector.detect(make_fire_frame())[0]
    blended = apply_verifier_confidence(detection, 1.0, 0.5)
    assert detection.confidence < blended.confidence <= 1.0


def test_onnx_verifier_rejects_empty_roi_without_model_load():
    verifier = OnnxFireVerifier.__new__(OnnxFireVerifier)
    assert verifier.predict(np.zeros((0, 0, 3))) == 0.0


def test_alert_tracker_confirms_then_holds():
    tracker = AlertTracker(min_consecutive_frames=3, hold_sec=2.0)
    assert not tracker.update(0.0, True)
    assert not tracker.update(0.1, True)
    assert tracker.update(0.2, True)
    assert tracker.update(0.3, False)
    assert tracker.update(2.2, False)
    assert not tracker.update(2.3, False)


def test_alert_tracker_requires_new_confirmation_after_expiry():
    tracker = AlertTracker(min_consecutive_frames=3, hold_sec=1.0)
    for now in (0.0, 0.1, 0.2):
        tracker.update(now, True)
    assert tracker.active
    assert not tracker.update(1.5, False)
    assert not tracker.update(1.6, True)
    assert not tracker.update(1.7, True)
    assert tracker.update(1.8, True)

