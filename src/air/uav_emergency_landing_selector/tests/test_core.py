import math
import unittest

from uav_emergency_landing_selector.core import (
    LandingConstraints,
    LandingSite,
    LandingSiteSelector,
    UAVState,
)


class LandingSelectorTests(unittest.TestCase):
    def setUp(self):
        self.uav = UAVState(0.0, 0.0, 30.0, 80.0, 3.0)
        self.safe = LandingSite("safe", 12.0, 5.0, 4.0, 0.12, 0.15, 0.03, 0.95)
        self.sloped = LandingSite("sloped", 5.0, 2.0, 22.0, 0.1, 0.1, 0.0, 0.98)

    def test_selects_eligible_site(self):
        decision = LandingSiteSelector().select(self.uav, [self.sloped, self.safe])
        self.assertEqual(decision.selected_site_id, "safe")
        self.assertEqual(decision.state, "SELECTED")

    def test_rejects_excessive_slope(self):
        evaluation = LandingSiteSelector().evaluate(self.uav, self.sloped)
        self.assertFalse(evaluation.eligible)
        self.assertIn("slope", evaluation.rejection_reasons)

    def test_rejects_people_risk(self):
        crowded = LandingSite("crowded", 4.0, 4.0, 2.0, 0.1, 0.0, 0.9, 0.9)
        decision = LandingSiteSelector().select(self.uav, [crowded])
        self.assertEqual(decision.state, "NO_SAFE_SITE")

    def test_distance_is_considered(self):
        far = LandingSite("far", 200.0, 0.0, 2.0, 0.1, 0.0, 0.0, 0.9)
        evaluation = LandingSiteSelector().evaluate(self.uav, far)
        self.assertIn("distance", evaluation.rejection_reasons)

    def test_low_battery_rejects_energy_margin(self):
        uav = UAVState(0.0, 0.0, 30.0, 20.0, 0.0)
        site = LandingSite("edge", 60.0, 0.0, 2.0, 0.1, 0.0, 0.0, 0.9)
        evaluation = LandingSiteSelector().evaluate(uav, site)
        self.assertIn("energy_margin", evaluation.rejection_reasons)

    def test_nan_is_rejected(self):
        bad = UAVState(math.nan, 0.0, 30.0, 80.0, 0.0)
        with self.assertRaises(ValueError):
            LandingSiteSelector().select(bad, [self.safe])

    def test_empty_candidate_list(self):
        self.assertEqual(LandingSiteSelector().select(self.uav, []).state, "NO_CANDIDATE")


if __name__ == "__main__":
    unittest.main()
