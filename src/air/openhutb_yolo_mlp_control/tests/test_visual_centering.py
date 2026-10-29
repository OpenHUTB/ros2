"""Deterministic controller checks; no ROS or simulator required."""
import math
import unittest

from openhutb_yolo_mlp_control.visual_centering import (
    Box, CenteringController, Settings,
)


def box(x=480, y=240, label='car', score=0.9):
    return Box(x, y, 80, 100, score, label)


class ControllerTests(unittest.TestCase):
    def ready(self, yaw=0, x=480):
        c = CenteringController()
        c.update_pose(0, 0, -3, yaw, 0)
        c.update_detections([box(x)], 640, 480, 0)
        return c

    def test_waits_for_both_inputs(self):
        c = CenteringController()
        self.assertEqual(c.step(0)[0], (0, 0, 0))
        c.update_pose(0, 0, -3, 0, 0)
        self.assertEqual(c.step(0)[1], 'target_lost')

    def test_target_right_moves_body_right(self):
        cmd, status, error = self.ready().step(0)
        self.assertAlmostEqual(cmd[1], 0.3)
        self.assertEqual(cmd[0], 0)
        self.assertEqual(status, 'tracking')
        self.assertEqual(error, 0.5)

    def test_target_left_moves_left(self):
        c = self.ready(x=160)
        self.assertLess(c.step(0)[0][1], 0)

    def test_body_right_rotates_into_ned(self):
        cmd = self.ready(math.pi/2).step(0)[0]
        self.assertAlmostEqual(cmd[0], -0.3)
        self.assertAlmostEqual(cmd[1], 0)

    def test_deadband_and_speed_limit(self):
        c = self.ready(x=320)
        self.assertEqual(c.step(0)[1], 'centered')
        c = self.ready(x=640)
        self.assertEqual(c.step(0)[0][1], 0.4)

    def test_altitude_ned_sign_and_limit(self):
        c = self.ready()
        c.step(0)
        c.update_pose(0, 0, -2, 0, 0.1)
        self.assertEqual(c.step(0.1)[0][2], -0.3)

    def test_detection_and_odom_expire(self):
        c = self.ready()
        self.assertEqual(c.step(0.51)[1], 'odom_stale')
        c.update_pose(0, 0, -3, 0, 0.9)
        self.assertEqual(c.step(0.9)[1], 'target_lost')

    def test_empty_detection_stops_immediately(self):
        c = self.ready()
        c.update_detections([], 640, 480, 0.1)
        self.assertEqual(c.step(0.1)[0], (0, 0, 0))

    def test_invalid_detection_filtered(self):
        for b in (box(label='person'), box(score=0.1), box(float('nan')),
                  box(score=float('inf')), box(-10)):
            c = self.ready()
            c.update_detections([b], 640, 480, 0.1)
            self.assertEqual(c.step(0.1)[1], 'target_lost')

    def test_nearest_association_and_reacquisition(self):
        c = self.ready()
        c.update_detections([box(470, score=0.6), box(100, score=0.99)], 640, 480, 0.1)
        self.assertGreater(c.target[0], 0)
        c.update_detections([box(100)], 640, 480, 0.2)
        self.assertIsNone(c.target)
        c.update_detections([box(100)], 640, 480, 1.2)
        self.assertLess(c.target[0], 0)

    def test_displacement_and_heading_guards(self):
        c = self.ready()
        c.step(0)
        c.update_pose(3, 0, -3, 0, 0.1)
        self.assertEqual(c.step(0.1)[1], 'displacement_limit')
        c.update_pose(0, 0, -3, 0.5, 0.2)
        self.assertEqual(c.step(0.2)[1], 'heading_changed')

    def test_invalid_pose_stops(self):
        c = self.ready()
        c.update_pose(float('nan'), 0, -3, 0, 0.1)
        self.assertEqual(c.step(0.1)[0], (0, 0, 0))

    def test_bad_settings_rejected(self):
        for kwargs in ({'gain': 0}, {'max_speed': float('nan')},
                       {'deadband': 1}, {'target_class': ''},
                       {'max_acceleration': -1}, {'max_acceleration': float('nan')}):
            with self.assertRaises(ValueError):
                Settings(**kwargs)

    def test_acceleration_ramp_and_reverse(self):
        c = CenteringController(Settings(gain=1.8, max_speed=0.8,
                                         max_acceleration=1.2))
        c.update_pose(0, 0, -3, 0, 0)
        c.update_detections([box()], 640, 480, 0)
        self.assertEqual(c.step(0)[1], 'tracking')
        for i in range(1, 7):
            self.assertAlmostEqual(c.step(i*.05)[0][1], i*.06)
        # Flip the selected target to exercise velocity reversal without
        # relying on the unrelated box association logic.
        c.target = (-0.5, 0, 0.9)
        self.assertAlmostEqual(c.step(.35)[0][1], .30)
        self.assertAlmostEqual(c.step(.35)[0][1], .30)

    def test_ramp_never_delays_stop_or_survives_target_loss(self):
        c = CenteringController(Settings(max_acceleration=1.2))
        c.update_pose(0, 0, -3, 0, 0)
        c.update_detections([box()], 640, 480, 0)
        c.step(0)
        self.assertGreater(c.step(.1)[0][1], 0)
        c.clear_detection()
        self.assertEqual(c.step(.15)[0], (0, 0, 0))
        c.update_detections([box()], 640, 480, .2)
        self.assertAlmostEqual(c.step(.2)[0][1], .06)
        c.target = (0, 0, .9)
        self.assertEqual(c.step(.25)[0], (0, 0, 0))

    def test_responsive_preset_converges_with_ramp(self):
        c = CenteringController(Settings(gain=1.8, max_speed=.8,
                                         max_acceleration=1.2))
        y = 0.0
        previous = 0.0
        for i in range(400):
            now = i*.05
            c.update_pose(0, y, -3, 0, now)
            c.update_detections([box(320*(1+(2-y)/6))], 640, 480, now)
            cmd, status, error = c.step(now)
            self.assertLessEqual(abs(cmd[1]), .8)
            if status == 'tracking':
                self.assertLessEqual(abs(cmd[1]-previous), .06000001)
            previous = cmd[1]
            y += cmd[1]*.05
        self.assertEqual(status, 'centered')
        self.assertLessEqual(abs(error), .08)

    def test_closed_loop_pinhole_model_converges(self):
        # Synthetic static target 6 m ahead, initially 2 m to the right.
        # This checks control sign and convergence, not AirSim dynamics.
        c = CenteringController()
        y = 0
        first = None
        for i in range(800):
            now = i*0.05
            c.update_pose(0, y, -3, 0, now)
            error = (2-y)/6  # 90 degree horizontal FOV
            c.update_detections([box(320*(1+error))], 640, 480, now)
            cmd, status, actual_error = c.step(now)
            first = actual_error if first is None else first
            y += cmd[1]*0.05
        self.assertEqual(status, 'centered')
        self.assertLess(abs(actual_error), abs(first)/3)


class DepthAndVerticalTests(unittest.TestCase):
    """Opt-in depth keeping and vertical centering; defaults stay unchanged."""

    def controller(self, **kwargs):
        c = CenteringController(Settings(**kwargs))
        c.update_pose(0, 0, -3, 0, 0)
        return c

    @staticmethod
    def sized(width, height, x=320, y=240):
        return Box(x, y, width, height, 0.9, 'car')

    def test_defaults_keep_no_forward_motion(self):
        c = self.controller()
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        self.assertEqual(c.step(0)[0][0], 0)
        self.assertEqual(c.last_forward_speed, 0)

    def test_far_target_moves_forward(self):
        c = self.controller(target_area=0.1)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)  # area ~ 0.026
        self.assertGreater(c.step(0)[0][0], 0)
        self.assertGreater(c.last_distance_error, 0)

    def test_near_target_backs_off(self):
        c = self.controller(target_area=0.1)
        c.update_detections([self.sized(300, 300)], 640, 480, 0)  # area ~ 0.29
        self.assertLess(c.step(0)[0][0], 0)
        self.assertLess(c.last_distance_error, 0)

    def test_forward_speed_limited(self):
        c = self.controller(target_area=0.1, distance_gain=50.0)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        self.assertAlmostEqual(c.step(0)[0][0], 0.5)  # max_forward_speed

    def test_forward_rotates_with_yaw(self):
        c = self.controller(target_area=0.1)
        c.update_pose(0, 0, -3, math.pi/2, 0)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        cmd = c.step(0)[0]
        self.assertAlmostEqual(cmd[0], 0, places=7)
        self.assertAlmostEqual(cmd[1], 0.5)

    def test_centered_needs_both_axes(self):
        far = self.controller(target_area=0.1)
        far.update_detections([self.sized(80, 100)], 640, 480, 0)
        self.assertEqual(far.step(0)[1], 'tracking')
        on_range = self.controller(target_area=0.1)
        on_range.update_detections([self.sized(175, 175)], 640, 480, 0)
        self.assertEqual(on_range.step(0)[1], 'centered')

    def test_vertical_centering_climbs_for_high_target(self):
        c = self.controller(vertical_gain=1.0)
        c.update_detections([self.sized(80, 100, y=100)], 640, 480, 0)
        self.assertLess(c.step(0)[0][2], 0)  # NED: negative z climbs

    def test_vertical_centering_descends_for_low_target(self):
        c = self.controller(vertical_gain=1.0)
        c.update_detections([self.sized(80, 100, y=400)], 640, 480, 0)
        self.assertGreater(c.step(0)[0][2], 0)

    def test_vertical_centering_overrides_altitude_hold(self):
        # Pose equals origin, so the altitude hold would output exactly zero.
        c = self.controller(vertical_gain=1.0)
        c.update_detections([self.sized(80, 100, y=400)], 640, 480, 0)
        self.assertNotEqual(c.step(0)[0][2], 0)

    def test_vertical_deadband_holds_level(self):
        c = self.controller(vertical_gain=1.0)
        c.update_detections([self.sized(80, 100, y=240)], 640, 480, 0)
        self.assertEqual(c.step(0)[0][2], 0)

    def test_altitude_limit_guard(self):
        c = self.controller(vertical_gain=1.0)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        c.step(0)
        c.update_pose(0, 0, -7, 0, 0.1)  # 4 m above the origin altitude
        self.assertEqual(c.step(0.1)[1], 'altitude_limit')

    def test_guard_zeroes_forward_component(self):
        c = self.controller(target_area=0.1)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        self.assertGreater(c.step(0)[0][0], 0)
        c.update_pose(3, 0, -3, 0, 0.1)
        self.assertEqual(c.step(0.1)[0], (0, 0, 0))
        self.assertEqual(c.last_forward_speed, 0)

    def test_target_loss_drops_seen_area(self):
        c = self.controller(target_area=0.1)
        c.update_detections([self.sized(80, 100)], 640, 480, 0)
        c.step(0)
        c.clear_detection()
        self.assertIsNone(c.seen_area)
        self.assertEqual(c.step(0.05)[1], 'target_lost')

    def test_new_settings_validation(self):
        for kwargs in ({'target_area': -0.1}, {'distance_gain': -1.0},
                       {'vertical_gain': -1.0}, {'distance_deadband': -0.1},
                       {'vertical_deadband': -0.1}, {'max_forward_speed': 0.0},
                       {'max_altitude_change': 0.0}, {'target_area': float('nan')},
                       {'distance_deadband': 1.0}, {'vertical_deadband': 1.0}):
            with self.assertRaises(ValueError):
                Settings(**kwargs)
        # Zero is the documented way to disable each opt-in feature.
        Settings(target_area=0.0, vertical_gain=0.0)


if __name__ == '__main__':
    unittest.main()
