#!/usr/bin/env python3
"""Tests for group-crossing behavior."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pedestrian_group_crossing import GroupCrossingPolicy


class GroupCrossingPolicyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = GroupCrossingPolicy(group_size=4)

    def test_waits_until_enough_members_arrive(self):
        self.assertTrue(self.policy.evaluate(2, 1.0, 8.0, False).wait)

    def test_crosses_when_group_is_cohesive_and_gap_is_safe(self):
        decision = self.policy.evaluate(4, 1.0, 8.0, False)
        self.assertFalse(decision.wait)
        self.assertEqual(decision.reason, "group_clear_to_cross")

    def test_spread_blocks_crossing(self):
        decision = self.policy.evaluate(4, 4.0, 8.0, False)
        self.assertTrue(decision.wait)
        self.assertEqual(decision.reason, "group_too_spread")

    def test_leader_waiting_blocks_crossing(self):
        decision = self.policy.evaluate(4, 1.0, 8.0, True)
        self.assertEqual(decision.reason, "leader_waiting")

    def test_small_gap_blocks_crossing(self):
        decision = self.policy.evaluate(4, 1.0, 1.0, False)
        self.assertEqual(decision.reason, "gap_below_threshold")

    def test_invalid_group_size_is_rejected(self):
        with self.assertRaises(ValueError):
            GroupCrossingPolicy(group_size=0).validate()


if __name__ == "__main__":
    unittest.main()
