import unittest

from uav_esc_telemetry_watch.core import ESCSample, ESCTelemetryWatch


class TestESCSpy(unittest.TestCase):
    def s(self, t, v=22.5, i=12.0, rpm=5000.0, temp=45.0, age=20.0):
        return ESCSample(t, v, i, rpm, temp, age)

    def test_ok(self):
        self.assertEqual(ESCTelemetryWatch().evaluate(self.s(0.0)).state, "OK")

    def test_low_voltage(self):
        r = ESCTelemetryWatch().evaluate(self.s(0.0, v=17.5))
        self.assertEqual(r.state, "ESC_LOST")
        self.assertEqual(r.recommendation, "MOTOR_FAILSAFE")

    def test_stale_frame(self):
        r = ESCTelemetryWatch().evaluate(self.s(0.0, age=300.0))
        self.assertEqual(r.state, "ESC_LOST")

    def test_overtemp(self):
        r = ESCTelemetryWatch().evaluate(self.s(0.0, temp=85.0))
        self.assertEqual(r.state, "ESC_LOST")

    def test_degraded(self):
        g = ESCTelemetryWatch()
        r = None
        for i in range(4):
            r = g.evaluate(self.s(i * 0.1, v=18.8, i=44.0, temp=79.0, age=140.0))
        self.assertIn(r.state, {"WATCH", "DEGRADED"})

    def test_invalid(self):
        with self.assertRaises(ValueError):
            ESCTelemetryWatch().evaluate(self.s(0.0, v=-1.0))


if __name__ == "__main__":
    unittest.main()
