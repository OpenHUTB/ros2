"""Fire detection primitives that do not depend on ROS 2.

The node layer only handles message conversion and temporal confirmation; all
image processing lives here so it can be tested with plain OpenCV.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

import cv2
import numpy as np


@dataclass
class Detection:
    """Axis-aligned flame candidate in image coordinates."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    confidence: float
    area_ratio: float
    center_x: float
    center_y: float

    @property
    def width(self) -> int:
        return self.x_max - self.x_min

    @property
    def height(self) -> int:
        return self.y_max - self.y_min

    @property
    def bbox(self) -> tuple[int, int, int, int]:
        return self.x_min, self.y_min, self.x_max, self.y_max


class AlertTracker:
    """Frame-to-frame confirmation and alarm hold behaviour."""

    def __init__(
        self,
        min_consecutive_frames: int = 3,
        hold_sec: float = 2.0,
    ) -> None:
        self.min_consecutive_frames = max(1, int(min_consecutive_frames))
        self.hold_sec = max(0.0, float(hold_sec))
        self.active = False
        self.consecutive_hits = 0
        self.last_detection_time: float | None = None

    def update(self, now: float, detected: bool) -> bool:
        if detected:
            self.consecutive_hits += 1
            self.last_detection_time = float(now)
        else:
            self.consecutive_hits = 0

        hold_active = (
            self.last_detection_time is not None
            and now - self.last_detection_time <= self.hold_sec
        )
        if self.active:
            self.active = hold_active
        else:
            self.active = (
                self.consecutive_hits >= self.min_consecutive_frames
                and hold_active
            )
        return self.active

    def reset(self) -> None:
        self.active = False
        self.consecutive_hits = 0
        self.last_detection_time = None


@dataclass
class FireDetectorConfig:
    """HSV thresholds and contour filters for the classical detector."""

    hsv_h_min: int = 0
    hsv_h_max: int = 35
    hsv_s_min: int = 90
    hsv_v_min: int = 110
    red_h_min: int = 160
    red_h_max: int = 179
    red_s_min: int = 110
    red_v_min: int = 90
    min_area_ratio: float = 0.0015
    max_area_ratio: float = 0.70
    min_confidence: float = 0.45
    morph_kernel: int = 5
    max_detections: int = 3
    min_aspect_ratio: float = 0.10
    max_aspect_ratio: float = 8.0


class HsvFireDetector:
    """Detect flame-coloured regions using HSV segmentation.

    The detector intentionally favours recall: the ROS node applies temporal
    confirmation and an optional neural verifier before raising an alarm.
    """

    def __init__(self, config: FireDetectorConfig | None = None) -> None:
        self.config = config or FireDetectorConfig()

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        if frame_bgr.ndim == 2:
            frame_bgr = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2BGR)
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] < 3:
            return []

        cfg = self.config
        height, width = frame_bgr.shape[:2]
        frame_area = float(height * width)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        lower_warm = np.array(
            [cfg.hsv_h_min, cfg.hsv_s_min, cfg.hsv_v_min], dtype=np.uint8
        )
        upper_warm = np.array(
            [cfg.hsv_h_max, 255, 255], dtype=np.uint8
        )
        lower_red = np.array(
            [cfg.red_h_min, cfg.red_s_min, cfg.red_v_min], dtype=np.uint8
        )
        upper_red = np.array([cfg.red_h_max, 255, 255], dtype=np.uint8)

        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_warm, upper_warm),
            cv2.inRange(hsv, lower_red, upper_red),
        )

        kernel_size = max(1, int(cfg.morph_kernel))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections: list[Detection] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area <= 0.0:
                continue
            area_ratio = area / frame_area
            if not (cfg.min_area_ratio <= area_ratio <= cfg.max_area_ratio):
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue
            aspect = w / float(h)
            if not (cfg.min_aspect_ratio <= aspect <= cfg.max_aspect_ratio):
                continue

            roi_mask = mask[y:y + h, x:x + w]
            roi_hsv = hsv[y:y + h, x:x + w]
            if np.any(roi_mask):
                mean_s = cv2.mean(roi_hsv, mask=roi_mask)[1]
                mean_v = cv2.mean(roi_hsv, mask=roi_mask)[2]
            else:
                mean_s = mean_v = 0.0
            color_score = 0.5 * (mean_s / 255.0) + 0.5 * (mean_v / 255.0)
            area_score = min(1.0, area_ratio / 0.01)
            confidence = min(
                0.99, 0.60 * area_score + 0.40 * color_score
            )

            if confidence < cfg.min_confidence:
                continue

            center_x = (x + w / 2.0) / width
            center_y = (y + h / 2.0) / height
            detections.append(
                Detection(
                    x_min=max(0, x),
                    y_min=max(0, y),
                    x_max=min(width, x + w),
                    y_max=min(height, y + h),
                    confidence=float(np.clip(confidence, 0.0, 1.0)),
                    area_ratio=float(area_ratio),
                    center_x=float(center_x),
                    center_y=float(center_y),
                )
            )

        detections.sort(key=lambda det: det.confidence, reverse=True)
        detections = self._non_max_suppression(detections, iou_threshold=0.35)
        return detections[:max(1, int(cfg.max_detections))]

    @staticmethod
    def _non_max_suppression(
        detections: Sequence[Detection], iou_threshold: float
    ) -> list[Detection]:
        kept: list[Detection] = []
        for det in detections:
            if all(_iou(det, other) < iou_threshold for other in kept):
                kept.append(det)
        return kept


def _iou(first: Detection, second: Detection) -> float:
    x1 = max(first.x_min, second.x_min)
    y1 = max(first.y_min, second.y_min)
    x2 = min(first.x_max, second.x_max)
    y2 = min(first.y_max, second.y_max)
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    if intersection == 0:
        return 0.0
    area_first = max(1, first.width * first.height)
    area_second = max(1, second.width * second.height)
    return intersection / float(area_first + area_second - intersection)


def annotate(
    frame_bgr: np.ndarray,
    detections: Sequence[Detection],
    alert: bool,
) -> np.ndarray:
    """Return a copy with candidate boxes and an alarm banner."""

    canvas = frame_bgr.copy()
    width = canvas.shape[1]
    colour = (0, 0, 255) if alert else (0, 165, 255)

    for det in detections:
        cv2.rectangle(
            canvas,
            (det.x_min, det.y_min),
            (det.x_max, det.y_max),
            colour,
            2,
        )
        label = f'FIRE {det.confidence * 100:.0f}%'
        (text_w, text_h), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
        )
        label_y = max(text_h + 6, det.y_min)
        cv2.rectangle(
            canvas,
            (det.x_min, label_y - text_h - 6),
            (det.x_min + text_w + 8, label_y + baseline),
            colour,
            -1,
        )
        cv2.putText(
            canvas,
            label,
            (det.x_min + 4, label_y - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    if alert:
        cv2.rectangle(canvas, (0, 0), (width - 1, 36), (0, 0, 255), -1)
        cv2.putText(
            canvas,
            'FIRE ALERT',
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return canvas


class OnnxFireVerifier:
    """Optional binary fire classifier loaded through ``cv2.dnn``.

    The model is expected to accept a square RGB image in [0, 1] and output
    either one logit/probability or a class score vector. Class index 1 is
    treated as fire by default. No model file is bundled with this package.
    """

    def __init__(
        self,
        model_path: str,
        input_size: int = 224,
        fire_class_index: int = 1,
    ) -> None:
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.input_size = max(32, int(input_size))
        self.fire_class_index = max(0, int(fire_class_index))

    def predict(self, roi_bgr: np.ndarray) -> float:
        if roi_bgr is None or roi_bgr.size == 0:
            return 0.0
        blob = cv2.dnn.blobFromImage(
            roi_bgr,
            scalefactor=1.0 / 255.0,
            size=(self.input_size, self.input_size),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        output = np.asarray(self.net.forward()).reshape(-1)
        if output.size == 0:
            return 0.0
        if output.size == 1:
            logit = float(np.clip(output[0], -30.0, 30.0))
            return float(1.0 / (1.0 + np.exp(-logit)))
        index = min(self.fire_class_index, output.size - 1)
        scores = output.astype(np.float64)
        scores -= np.max(scores)
        probabilities = np.exp(scores) / np.sum(np.exp(scores))
        return float(probabilities[index])


def apply_verifier_confidence(
    detection: Detection,
    model_confidence: float,
    model_weight: float = 0.5,
) -> Detection:
    weight = float(np.clip(model_weight, 0.0, 1.0))
    combined = (1.0 - weight) * detection.confidence
    combined += weight * float(np.clip(model_confidence, 0.0, 1.0))
    return replace(detection, confidence=float(np.clip(combined, 0.0, 1.0)))
