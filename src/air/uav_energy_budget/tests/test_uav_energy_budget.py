import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_energy_budget.main import DEFAULT_INPUT, evaluate_mission


class UavEnergyBudgetTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))

    def test_sample_mission_passes(self):
        rows, summary, errors = evaluate_mission(self.profile)
        self.assertEqual(errors, [])
        self.assertEqual(len(rows), 4)
        self.assertTrue(summary["feasible"])
        self.assertGreater(summary["total_energy_wh"], 0)

    def test_headwind_above_airspeed_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["headwind_ms"] = 20.0
        _, _, errors = evaluate_mission(profile)
        self.assertTrue(any("headwind" in error for error in errors))

    def test_payload_increases_energy(self):
        profile = copy.deepcopy(self.profile)
        _, baseline, _ = evaluate_mission(profile)
        profile["aircraft"]["payload_kg"] += 2.0
        _, heavier, _ = evaluate_mission(profile)
        self.assertGreater(heavier["total_energy_wh"], baseline["total_energy_wh"])
        self.assertLess(
            heavier["max_cruise_distance_m"],
            baseline["max_cruise_distance_m"],
        )

    def test_max_cruise_distance_uses_effective_mass_adjusted_power(self):
        _, summary, errors = evaluate_mission(self.profile)
        self.assertEqual(errors, [])
        aircraft = self.profile["aircraft"]
        battery = self.profile["battery"]
        mass_factor = (
            (aircraft["empty_mass_kg"] + aircraft["payload_kg"])
            / aircraft["empty_mass_kg"]
        ) ** aircraft["mass_power_exponent"]
        expected = (
            battery["capacity_wh"]
            * (battery["efficiency"] - battery["reserve_fraction"])
            / (aircraft["cruise_power_w"] * mass_factor)
            * 3600.0
            * (aircraft["cruise_airspeed_ms"] - self.profile["headwind_ms"])
        )
        self.assertAlmostEqual(summary["max_cruise_distance_m"], expected, places=3)

    def test_reserve_failure_is_reported(self):
        profile = copy.deepcopy(self.profile)
        profile["segments"].append({"name": "long_return", "kind": "cruise", "distance_m": 50000.0})
        _, summary, errors = evaluate_mission(profile)
        self.assertEqual(errors, [])
        self.assertFalse(summary["feasible"])

    def test_invalid_segment_kind_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["segments"][0]["kind"] = "teleport"
        _, _, errors = evaluate_mission(profile)
        self.assertTrue(any("kind" in error for error in errors))


if __name__ == "__main__":
    unittest.main(verbosity=2)
