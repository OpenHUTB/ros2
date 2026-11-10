import unittest

from uav_baro_altitude_drift.core import (
    AltSample,
    BaroAltitudeDriftWatch,
    BaroWatchConfig,
)


class TestBaroDrift(unittest.TestCase):
    def s(self, t, baro=20.0, gps=20.0, radar=20.0, vs=0.0, p=1005.0, temp=22.0):
        return AltSample(t, baro, gps, radar, vs, p, temp)

    def test_calm(self):
        r = BaroAltitudeDriftWatch().evaluate(self.s(0.0))
        self.assertEqual(r.state, "CALM")

    def test_watch_hysteresis(self):
        g = BaroAltitudeDriftWatch()
        rs = [g.evaluate(self.s(i * 0.1, baro=36.0)) for i in range(4)]
        self.assertEqual(rs[0].state, "CALM")
        self.assertEqual(rs[-1].state, "WATCH")

    def test_drift_state(self):
        g = BaroAltitudeDriftWatch()
        r = None
        for i in range(6):
            r = g.evaluate(self.s(i * 0.1, baro=42.0, radar=20.0))
        self.assertIn(r.state, {"DRIFT", "BARO_LOST"})

    def test_hard_lost(self):
        r = BaroAltitudeDriftWatch().evaluate(self.s(0.0, baro=50.0, p=995.0))
        self.assertEqual(r.state, "BARO_LOST")

    def test_invalid_pressure(self):
        with self.assertRaises(ValueError):
            BaroAltitudeDriftWatch().evaluate(self.s(0.0, p=-1.0))

    def test_bad_thresholds(self):
        with self.assertRaises(ValueError):
            BaroWatchConfig(watch_risk=0.9, drift_risk=0.5, lost_risk=0.95)


if __name__ == "__main__":
    unittest.main()
