import unittest

from uav_mission_safety_manager.demo_adapter import DemoAdapter


class TestDemoAdapter(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.adapter = DemoAdapter(clock=lambda: self.now)

    def test_moves_at_requested_speed_and_hover_stops(self):
        self.adapter.submit('goto', target=(4., 0., 0.), speed=2.)
        self.now = 1.
        self.assertEqual(self.adapter.sample().result()['position'], (2., 0., 0.))
        self.adapter.submit('hover')
        self.now = 3.
        data = self.adapter.sample().result()
        self.assertEqual(data['position'], (2., 0., 0.))
        self.assertEqual(data['velocity'], (0., 0., 0.))

    def test_zero_velocity_correction_at_current_position_is_stationary(self):
        self.adapter.submit('velocity', velocity=(0., 0., 0.), duration=0.3)
        self.now = 1.
        self.assertEqual(self.adapter.sample().result()['velocity'], (0., 0., 0.))

    def test_takeoff_and_land_are_confirmed_by_demo_telemetry(self):
        self.adapter.submit('takeoff')
        self.assertFalse(self.adapter.sample().result()['landed'])
        self.now = 1.
        self.assertEqual(self.adapter.sample().result()['position'], (0., 0., -2.))
        self.adapter.submit('land')
        self.now = 2.
        self.assertTrue(self.adapter.sample().result()['landed'])

    def test_unknown_command_rejected(self):
        with self.assertRaises(ValueError):
            self.adapter.submit('unknown')
