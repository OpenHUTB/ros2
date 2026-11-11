import unittest

from uav_control_authority_guard.core import (
    ControlAuthorityConfig,
    ControlAuthorityGuard,
    ControlSample,
)


class ControlAuthorityTests(unittest.TestCase):
    def sample(
        self,
        t=0.0,
        cmd_r=20.0,
        act_r=19.0,
        cmd_p=5.0,
        act_p=4.8,
        saturation=0.1,
        integrator=0.1,
        voltage=3.9,
    ):
        return ControlSample(t, cmd_r, act_r, cmd_p, act_p, saturation, integrator, voltage)

    def test_nominal_response(self):
        result = ControlAuthorityGuard().evaluate(self.sample())
        self.assertEqual(result.state, "NOMINAL")

    def test_saturation_risk(self):
        guard = ControlAuthorityGuard()
        result = None
        for i in range(4):
            result = guard.evaluate(self.sample(i * 0.1, 40, 20, 25, 10, 0.92, 0.75, 3.4))
        self.assertIn(result.state, {"SATURATION_RISK", "CRITICAL_AUTHORITY"})

    def test_immediate_critical_authority(self):
        result = ControlAuthorityGuard().evaluate(
            self.sample(0.0, 50, 12, 35, 8, 0.96, 0.9, 3.2)
        )
        self.assertEqual(result.state, "CRITICAL_AUTHORITY")

    def test_recommendation(self):
        result = ControlAuthorityGuard().evaluate(
            self.sample(0.0, 50, 12, 35, 8, 0.96, 0.9, 3.2)
        )
        self.assertEqual(result.recommendation, "STABILIZE_AND_DESCEND")

    def test_invalid_saturation(self):
        with self.assertRaises(ValueError):
            ControlAuthorityGuard().evaluate(self.sample(saturation=1.2))

    def test_non_increasing_time(self):
        guard = ControlAuthorityGuard()
        guard.evaluate(self.sample(1.0))
        with self.assertRaises(ValueError):
            guard.evaluate(self.sample(1.0))

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            ControlAuthorityConfig(reduced_risk=0.7, saturation_risk=0.6)

    def test_authority_ratio_is_bounded(self):
        result = ControlAuthorityGuard().evaluate(self.sample(cmd_r=0.0, act_r=100.0))
        self.assertLessEqual(result.authority_ratio, 1.0)


if __name__ == "__main__":
    unittest.main()
