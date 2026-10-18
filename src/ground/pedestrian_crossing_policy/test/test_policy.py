#!/usr/bin/env python3
"""Unit tests for the crossing policy."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pedestrian_crossing_policy import CrossingDecision, CrossingPolicy


class CrossingPolicyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = CrossingPolicy()

    def test_far_vehicle_does_not_block(self):
        self.assertFalse(self.policy.should_wait(20.0, 30.0))

    def test_approaching_vehicle_inside_ttc_blocks(self):
        self.assertTrue(self.policy.should_wait(10.0, 10.0))

    def test_slow_approaching_vehicle_does_not_block(self):
        self.assertFalse(self.policy.should_wait(10.0, 1.0))

    def test_close_stationary_vehicle_blocks(self):
        self.assertTrue(self.policy.should_wait(1.0, 0.0))

    def test_receding_vehicle_does_not_block(self):
        self.assertFalse(self.policy.should_wait(5.0, -2.0))

    def test_invalid_distance_order_is_rejected(self):
        with self.assertRaises(ValueError):
            CrossingPolicy(
                min_vehicle_distance=20.0,
                max_vehicle_distance=10.0,
            ).validate()

    def test_decision_contains_reason_and_ttc(self):
        decision = CrossingDecision.evaluate(self.policy, 10.0, 10.0)
        self.assertTrue(decision.wait)
        self.assertEqual(decision.reason, "ttc_below_threshold")
        self.assertAlmostEqual(decision.time_to_collision, 1.0)


if __name__ == "__main__":
    unittest.main()
