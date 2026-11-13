import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_payload_cg_envelope.main import DEFAULT_INPUT, check_cg  # noqa: E402


class CgTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(
            DEFAULT_INPUT.read_text(encoding="utf-8")
        )

    def test_sample_pass(self):
        result, errors = check_cg(self.profile)
        self.assertEqual(errors, [])
        self.assertTrue(result["feasible"])
        self.assertTrue(result["cg_z_m_is_informational"])

    def test_forward_payload_shifts_cg_forward(self):
        baseline, errors = check_cg(self.profile)
        self.assertEqual(errors, [])
        profile = copy.deepcopy(self.profile)
        profile["payloads"][1]["cg_m"][0] = 0.5
        shifted, errors = check_cg(profile)
        self.assertEqual(errors, [])
        self.assertGreater(shifted["cg_x_m"], baseline["cg_x_m"])

    def test_outside_longitudinal_limit_fails(self):
        profile = copy.deepcopy(self.profile)
        profile["payloads"][1]["cg_m"][0] = 10
        result, errors = check_cg(profile)
        self.assertEqual(errors, [])
        self.assertFalse(result["feasible"])

    def test_excess_lateral_offset_fails(self):
        profile = copy.deepcopy(self.profile)
        profile["payloads"][0]["cg_m"][1] = 1
        result, errors = check_cg(profile)
        self.assertEqual(errors, [])
        self.assertFalse(result["feasible"])

    def test_zero_payload_mass_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["payloads"][0]["mass_kg"] = 0
        result, errors = check_cg(profile)
        self.assertEqual(result, {})
        self.assertTrue(errors)

    def test_overlarge_item_mass_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["empty"]["mass_kg"] = 1e308
        profile["payloads"][0]["mass_kg"] = 1e308
        result, errors = check_cg(profile)
        self.assertEqual(result, {})
        self.assertTrue(any("must not exceed" in error for error in errors))

    def test_nonfinite_combined_moment_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        profile["empty"]["cg_m"][0] = 1e308
        result, errors = check_cg(profile)
        self.assertEqual(result, {})
        self.assertTrue(
            any("combined mass and moments must be finite" in error for error in errors)
        )

    def test_low_longitudinal_margin_warns(self):
        baseline, errors = check_cg(self.profile)
        self.assertEqual(errors, [])
        profile = copy.deepcopy(self.profile)
        profile["limits"]["cg_x_max_m"] = baseline["cg_x_m"] + 0.0025
        result, errors = check_cg(profile)
        self.assertEqual(errors, [])
        self.assertTrue(result["feasible"])
        self.assertTrue(
            any("longitudinal CG margin below 5 mm" in warning for warning in result["warnings"])
        )

    def test_excess_total_mass_is_rejected(self):
        profile = copy.deepcopy(self.profile)
        for payload in profile["payloads"]:
            payload["mass_kg"] = 9000
        result, errors = check_cg(profile)
        self.assertEqual(result, {})
        self.assertTrue(
            any("total_mass_kg must not exceed 10000" in error for error in errors)
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
