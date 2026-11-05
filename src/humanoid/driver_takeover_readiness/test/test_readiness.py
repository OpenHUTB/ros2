#!/usr/bin/env python3
"""Tests for takeover readiness."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from driver_takeover_readiness import TakeoverReadinessPolicy


class ReadinessTest(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = TakeoverReadinessPolicy()

    def test_ready_driver_has_high_score(self):
        result = self.policy.evaluate(True, True, True, 2.0, 0.2)
        self.assertTrue(result.ready)
        self.assertEqual(result.level, "ready")

    def test_hands_off_is_critical(self):
        result = self.policy.evaluate(True, False, True, 2.0, 0.0)
        self.assertFalse(result.ready)
        self.assertEqual(result.level, "critical")

    def test_long_gaze_off_road_is_critical(self):
        result = self.policy.evaluate(False, True, True, 2.0, 3.0)
        self.assertIn("critical_condition", result.reasons)

    def test_short_orientation_time_reduces_score(self):
        early = self.policy.evaluate(True, True, True, 0.1, 0.0)
        later = self.policy.evaluate(True, True, True, 2.0, 0.0)
        self.assertLess(early.score, later.score)

    def test_feet_not_ready_is_explained(self):
        result = self.policy.evaluate(True, True, False, 2.0, 0.0)
        self.assertIn("feet_not_ready", result.reasons)

    def test_invalid_weights_are_rejected(self):
        with self.assertRaises(ValueError):
            TakeoverReadinessPolicy(gaze_weight=0.5, hands_weight=0.5, feet_weight=0.5, alert_weight=0.5).validate()


if __name__ == "__main__":
    unittest.main()
