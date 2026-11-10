import unittest

from uav_motor_temp_guard.core import MotorSample, MotorTempGuard, TempConfig


class TestMotorTemp(unittest.TestCase):
    def test_cool_at_start(self):
        g = MotorTempGuard()
        r = g.evaluate(MotorSample(0.0, 10.0, 25.0, 4000.0))
        self.assertEqual(r.state, "COOL")

    def test_overtemp_on_sustained_high_current(self):
        g = MotorTempGuard()
        r = None
        for i in range(200):
            r = g.evaluate(MotorSample(i * 0.1, 40.0, 25.0, 8000.0))
        self.assertGreater(r.estimated_temp_c, 75.0)
        self.assertIn(r.state, {"HOT", "OVERTEMP"})

    def test_cooldown(self):
        g = MotorTempGuard()
        g.evaluate(MotorSample(0.0, 40.0, 25.0, 8000.0))
        for i in range(1, 300):
            g.evaluate(MotorSample(i * 0.1, 5.0, 25.0, 2000.0))
        # after long low-current cool-down, temp should drop
        r = g.evaluate(MotorSample(30.0, 5.0, 25.0, 2000.0))
        self.assertLess(r.estimated_temp_c, 50.0)

    def test_negative_current_rejected(self):
        with self.assertRaises(ValueError):
            MotorTempGuard().evaluate(MotorSample(0.0, -1.0, 25.0, 0.0))


if __name__ == "__main__":
    unittest.main()
