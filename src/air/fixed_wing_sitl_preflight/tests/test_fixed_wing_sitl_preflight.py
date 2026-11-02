import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fixed_wing_sitl_preflight.main import DEFAULT_INPUT, validate_profile


class FixedWingPreflightTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(DEFAULT_INPUT.read_text(encoding="utf-8"))

    def test_valid_profile_passes(self):
        errors, warnings = validate_profile(self.profile)
        self.assertEqual(errors, [])
        self.assertEqual(warnings, [])

    def test_missing_airspeed_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["sensors"]["airspeed"] = False
        errors, _ = validate_profile(profile)
        self.assertTrue(any("airspeed" in error for error in errors))

    def test_duplicate_actuator_channels_are_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["actuators"]["elevator"] = profile["actuators"]["aileron_left"]
        errors, _ = validate_profile(profile)
        self.assertTrue(any("unique" in error for error in errors))

    def test_overlapping_ports_are_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["connection"]["mavlink_port"] = profile["connection"]["sitl_port"]
        errors, _ = validate_profile(profile)
        self.assertTrue(any("must differ" in error for error in errors))

    def test_invalid_gps_origin_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["origin"]["latitude_deg"] = 91
        errors, _ = validate_profile(profile)
        self.assertTrue(any("latitude_deg" in error for error in errors))

    def test_low_battery_reserve_is_warned(self):
        profile = copy.deepcopy(self.profile)
        profile["failsafe"]["minimum_battery_percent"] = 15
        errors, warnings = validate_profile(profile)
        self.assertEqual(errors, [])
        self.assertTrue(any("reserve" in warning for warning in warnings))


if __name__ == "__main__":
    unittest.main(verbosity=2)
