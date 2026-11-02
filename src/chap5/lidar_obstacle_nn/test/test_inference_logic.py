"""Tests for LaserScan-style message processing without a ROS runtime."""

from __future__ import annotations

from types import SimpleNamespace
import unittest

import numpy as np

from lidar_obstacle_nn.scan_utils import classify_scan_message


class TestInferenceLogic(unittest.TestCase):
    def test_message_resampled_and_uses_confident_prediction(self) -> None:
        message = SimpleNamespace(ranges=[float('inf')] * 180)
        message.ranges[90] = 0.5
        observed_shapes = []

        def predict(scan):
            observed_shapes.append(scan.shape)
            return [0.05, 0.90, 0.05]

        prediction = classify_scan_message(
            message,
            predict_proba=predict,
            input_size=360,
        )

        self.assertEqual(observed_shapes, [(360,)])
        self.assertEqual(prediction.class_id, 1)
        self.assertEqual(prediction.class_name, 'far')
        self.assertAlmostEqual(prediction.confidence, 0.90)
        self.assertFalse(prediction.used_fallback)

    def test_low_confidence_falls_back_to_range_rule(self) -> None:
        message = SimpleNamespace(ranges=[0.5] + [float('inf')] * 359)

        prediction = classify_scan_message(
            message,
            predict_proba=lambda _scan: [0.34, 0.33, 0.33],
            input_size=360,
            danger_distance=1.0,
            far_distance=3.0,
            fallback_confidence=0.45,
        )

        self.assertEqual(prediction.class_id, 0)
        self.assertEqual(prediction.class_name, 'near')
        self.assertTrue(prediction.used_fallback)
        self.assertAlmostEqual(prediction.confidence, 0.34)

    def test_invalid_ranges_are_sanitized_to_free(self) -> None:
        message = SimpleNamespace(ranges=[float('nan'), float('inf')])
        captured = {}

        def predict(scan):
            captured['scan'] = scan
            return [float('nan'), float('inf'), 0.0]

        prediction = classify_scan_message(
            message,
            predict_proba=predict,
            input_size=360,
            fallback_confidence=0.45,
        )

        self.assertEqual(captured['scan'].shape, (360,))
        self.assertTrue(np.isfinite(captured['scan']).all())
        self.assertEqual(float(captured['scan'].min()), 30.0)
        self.assertEqual(prediction.class_id, 2)
        self.assertEqual(prediction.class_name, 'free')
