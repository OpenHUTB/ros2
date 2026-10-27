"""Synthetic depth tests; no AirSim server required."""
import math
import unittest
from types import SimpleNamespace
from avoidance import (Command, decode_unity_depth, detour_point, insert_detour,
                       plan, route_from_start, check_new_collision, smooth_yaw_rate)


class CollisionTests(unittest.TestCase):
    def report(self, collided, timestamp, name="Cube (65)"):
        return SimpleNamespace(has_collided=collided, time_stamp=timestamp,
                               object_name=name, penetration_depth=0.01)

    def test_old_ground_contact_is_ignored(self):
        check_new_collision(self.report(True, 100), 100)

    def test_no_contact_does_not_abort(self):
        check_new_collision(self.report(False, 0), 100)

    def test_new_contact_with_same_floor_still_aborts(self):
        with self.assertRaisesRegex(RuntimeError, r"Cube \(65\)"):
            check_new_collision(self.report(True, 101), 100)

    def test_first_collision_without_old_contact_aborts(self):
        with self.assertRaisesRegex(RuntimeError, "Obstacle"):
            check_new_collision(self.report(True, 101, "Obstacle"), 0)


class PlannerTests(unittest.TestCase):
    width, height = 80, 40

    def test_unity_depth_is_decoded_to_metres(self):
        encoded = 0.5 ** 0.25
        decoded = decode_unity_depth([encoded], near=0.3, far=100.0)[0]
        self.assertAlmostEqual(decoded, (0.5 / 1.003 + 0.003) * 100, places=2)

    def test_srgb_depth_round_trip_for_near_pillar(self):
        for distance in (0.5, 1, 2, 3, 10):
            shader = ((distance / 1000 - 0.3 / 1000) * 1.0003) ** 0.25
            captured = 1.055 * shader ** (1 / 2.4) - 0.055
            decoded = decode_unity_depth([captured], srgb=True)[0]
            self.assertAlmostEqual(decoded, distance, places=5)

    def test_pillar_generates_detour_even_when_front_is_blocked(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(32, 48):
                image[row * self.width + col] = 3.5
        cmd = self.command(image)
        self.assertEqual(cmd.reason, "curve_around_obstacle")
        self.assertGreater(cmd.speed, 0)
        self.assertLess(cmd.front_clearance, 2)
        self.assertGreater(cmd.clearance, 2)
        self.assertGreater(abs(cmd.steer_angle), math.radians(10))

    def test_yaw_reversal_is_rate_limited(self):
        self.assertAlmostEqual(smooth_yaw_rate(10, -20, 0.2), 7.6)
        self.assertLessEqual(smooth_yaw_rate(0, 30, 1), 6)

    def test_faster_curve_respects_requested_speed_limit(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(35, 45):
                image[row * self.width + col] = 5.5
        slow = plan(image, self.width, self.height, 90, 0, max_speed=0.4,
                    stop_distance=3, safety_margin=1.5)
        fast = plan(image, self.width, self.height, 90, 0, max_speed=1.2,
                    stop_distance=3, safety_margin=1.5)
        self.assertGreater(fast.speed, slow.speed)
        self.assertAlmostEqual(fast.speed, 1.2)
        self.assertGreater(fast.clearance, 3)

    def test_near_stop_distance_selects_wide_detour_not_creeping_forward(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(35, 45):
                image[row * self.width + col] = 5.5
        cmd = plan(image, self.width, self.height, 90, 0, max_speed=0.4,
                   stop_distance=3, safety_margin=1.5)
        self.assertEqual(cmd.reason, "curve_around_obstacle")
        self.assertGreaterEqual(cmd.speed, 0.18)
        self.assertGreater(abs(cmd.steer_angle), math.radians(12))
        self.assertGreater(cmd.clearance, 3)

    def test_locked_side_survives_small_goal_direction_changes(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(32, 48):
                image[row * self.width + col] = 3.5
        for angle in (-0.03, 0.03):
            cmd = plan(image, self.width, self.height, 90, angle,
                       preferred_angle=-0.4, avoidance_side=-1)
            self.assertLess(cmd.steer_angle, 0)
            self.assertGreater(cmd.speed, 0)

    def frame(self, fill=20.0):
        return [fill] * (self.width * self.height)

    def command(self, image, goal=0):
        return plan(image, self.width, self.height, 90, goal)

    def test_clear_corridor_advances(self):
        cmd = self.command(self.frame())
        self.assertGreater(cmd.speed, 0)
        self.assertLess(abs(cmd.yaw_rate), 3)

    def test_obstacle_inside_safety_buffer_does_not_advance(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(30, self.width):
                image[row * self.width + col] = 1.0
        cmd = self.command(image)
        self.assertEqual(cmd.speed, 0)
        self.assertEqual(cmd.reason, "blocked_hover")

    def test_extra_margin_rejects_a_narrow_gap(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(self.width):
                if col < 26 or col > 53:
                    image[row * self.width + col] = 3.5
        tight = plan(image, self.width, self.height, 90, 0,
                     stop_distance=3, safety_margin=0, vehicle_radius=0.6)
        buffered = plan(image, self.width, self.height, 90, 0,
                        stop_distance=3, safety_margin=1, vehicle_radius=0.6)
        self.assertGreater(tight.speed, 0)
        self.assertLess(buffered.speed, tight.speed)

    def test_fully_blocked_hovers(self):
        cmd = self.command(self.frame(0.7))
        self.assertEqual((cmd.speed, cmd.yaw_rate), (0, 0))

    def test_invalid_camera_aborts(self):
        for image in ([], self.frame(float("nan")), self.frame(0)):
            with self.assertRaises(ValueError):
                self.command(image)

    def test_invalid_front_pixels_do_not_allow_forward_motion(self):
        image = self.frame()
        for row in range(self.height):
            image[row * self.width + self.width // 2] = float("nan")
        self.assertEqual(self.command(image).speed, 0)

    def test_goal_behind_turns_in_place(self):
        cmd = self.command(self.frame(), math.pi)
        self.assertNotEqual(cmd.reason, "turn_to_goal")
        self.assertGreater(cmd.speed, 0)

    def test_route_is_rectangular(self):
        self.assertEqual(route_from_start(10, 20, 12, 6),
                         [(22, 20), (22, 26), (10, 26), (10, 20)])

    def test_scan_result_inserts_detour_before_original_target(self):
        route = [(12, 0), (12, 6), (0, 6), (0, 0)]
        insert_detour(route, 0, 3, -2)
        self.assertEqual(route[0], (3, -2))
        self.assertEqual(route[1], (12, 0))

    def test_turning_scan_becomes_world_waypoint(self):
        cmd = Command(0, -18, 5, "turn_to_clear_sector", math.radians(-20))
        x, y = detour_point(0, 0, 0, cmd)
        self.assertGreater(x, 0)
        self.assertLess(y, 0)


if __name__ == "__main__":
    unittest.main()
