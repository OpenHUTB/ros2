import math
import unittest

from uav_dynamic_geofence_guard.core import (
    DynamicZone,
    GeofenceConfig,
    GeofenceGuard,
    UAVMotion,
)


class DynamicGeofenceTests(unittest.TestCase):
    def setUp(self):
        self.uav = UAVMotion(0.0, 0.0, 2.0, 0.0)

    def test_direct_path_is_critical(self):
        zone = DynamicZone("z", 20.0, 0.0, 3.0)
        result = GeofenceGuard().assess(self.uav, [zone])
        self.assertEqual(result.state, "CRITICAL")
        self.assertAlmostEqual(result.predicted_clearance_m, -3.0)

    def test_far_zone_is_clear(self):
        zone = DynamicZone("z", 0.0, 30.0, 3.0)
        result = GeofenceGuard().assess(self.uav, [zone])
        self.assertEqual(result.state, "CLEAR")

    def test_warning_margin(self):
        zone = DynamicZone("z", 20.0, 7.5, 3.0)
        result = GeofenceGuard().assess(self.uav, [zone])
        self.assertEqual(result.state, "WARNING")

    def test_crossing_moving_zone(self):
        zone = DynamicZone("z", 10.0, -8.0, 2.0, vy=1.5)
        result = GeofenceGuard().assess(self.uav, [zone])
        self.assertLess(result.predicted_clearance_m, 1.5)
        self.assertGreater(result.time_to_closest_s, 0.0)

    def test_inside_zone_is_breach(self):
        zone = DynamicZone("z", 1.0, 0.0, 3.0)
        result = GeofenceGuard().assess(self.uav, [zone])
        self.assertEqual(result.state, "BREACH")

    def test_inactive_zone_ignored(self):
        zone = DynamicZone("z", 1.0, 0.0, 3.0, active=False)
        self.assertEqual(GeofenceGuard().assess(self.uav, [zone]).state, "CLEAR")

    def test_nearest_priority_zone_is_reported(self):
        warning = DynamicZone("warning", 20.0, 4.0, 3.0)
        critical = DynamicZone("critical", 20.0, 0.0, 3.0)
        result = GeofenceGuard().assess(self.uav, [warning, critical])
        self.assertEqual(result.limiting_zone_id, "critical")

    def test_invalid_zone_radius(self):
        with self.assertRaises(ValueError):
            GeofenceGuard().assess(self.uav, [DynamicZone("z", 1.0, 1.0, 0.0)])

    def test_invalid_margin_config(self):
        with self.assertRaises(ValueError):
            GeofenceConfig(warning_margin_m=1.0, critical_margin_m=2.0)


if __name__ == "__main__":
    unittest.main()
