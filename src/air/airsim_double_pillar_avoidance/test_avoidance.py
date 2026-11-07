"""Synthetic depth and state-machine tests; no AirSim server required."""
import math
import json
import tempfile
from pathlib import Path
import unittest
from types import SimpleNamespace
from avoidance import (SlalomState, ProgressWatchdog, create_run_directory, record_event,
                       decode_unity_depth, plan, route_from_start, check_new_collision,
                       smooth_yaw_rate, parse_args, MAX_YAW_RATE, RELEASE_FRAMES,
                       SIDE_COOLDOWN_FRAMES)


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


class PlannerTests(unittest.TestCase):
    width, height = 80, 40

    def frame(self, fill=20.0):
        return [fill] * (self.width * self.height)

    def command(self, image, goal=0):
        return plan(image, self.width, self.height, 90, goal)

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

    def test_pillar_selects_curve_when_front_is_blocked(self):
        image = self.frame()
        for row in range(self.height):
            for col in range(32, 48):
                image[row * self.width + col] = 3.5
        cmd = self.command(image)
        self.assertEqual(cmd.reason, "curve_around_obstacle")
        self.assertGreater(cmd.speed, 0)
        self.assertGreater(abs(cmd.steer_angle), math.radians(10))

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

    def test_clear_corridor_advances(self):
        cmd = self.command(self.frame())
        self.assertGreater(cmd.speed, 0)
        self.assertLess(abs(cmd.yaw_rate), 3)

    def test_fully_blocked_hovers(self):
        cmd = self.command(self.frame(0.7))
        self.assertEqual((cmd.speed, cmd.yaw_rate), (0, 0))

    def test_invalid_camera_aborts(self):
        for image in ([], self.frame(float("nan")), self.frame(0)):
            with self.assertRaises(ValueError):
                self.command(image)

    def test_route_is_rectangular(self):
        self.assertEqual(route_from_start(10, 20, 12, 6),
                         [(22, 20), (22, 26), (10, 26), (10, 20)])

    def test_yaw_reversal_is_rate_limited(self):
        self.assertAlmostEqual(smooth_yaw_rate(10, -20, 0.2), 10 - 12.0 * 0.2)
        self.assertLessEqual(abs(smooth_yaw_rate(0, 30, 1)), MAX_YAW_RATE)

    def test_scan_rate_uses_shared_yaw_constant(self):
        # The reference bug: a hard-coded 15 deg/s scan rate exceeded the
        # smoothing limit and never took effect. MAX_YAW_RATE is now shared.
        self.assertLessEqual(MAX_YAW_RATE, 12.0)


class SlalomStateTests(unittest.TestCase):
    """The new behaviour: unlock the side after a pillar so an opposite
    pillar on the far side is bypassed the other way, producing an S path."""

    def test_first_pillar_locks_chosen_side(self):
        state = SlalomState()
        state.update("curve_around_obstacle", 0.5)
        self.assertEqual(state.side, 1)
        state.update("curve_around_obstacle", -0.5)
        self.assertEqual(state.side, 1)  # locked side is not flipped mid-obstacle

    def test_side_unlocks_after_enough_clear_frames(self):
        state = SlalomState()
        state.update("curve_around_obstacle", 0.5)
        cleared = False
        for _ in range(RELEASE_FRAMES):
            cleared = state.update("advance", 0.0)
        self.assertTrue(cleared)
        self.assertEqual(state.pillars_cleared, 1)
        self.assertEqual(state.side, 0)

    def test_second_pillar_on_opposite_side_gets_fresh_choice(self):
        state = SlalomState()
        state.update("curve_around_obstacle", 0.5)          # pillar 1 -> right
        for _ in range(RELEASE_FRAMES):
            state.update("advance", 0.0)                     # clear pillar 1
        # Cooldown keeps the side free while crossing the gap, then a pillar on
        # the left re-locks the opposite side instead of wobbling on the old one.
        for _ in range(SIDE_COOLDOWN_FRAMES):
            state.update("curve_around_obstacle", -0.5)
            self.assertEqual(state.side, 0)
        state.update("curve_around_obstacle", -0.5)
        self.assertEqual(state.side, -1)

    def test_full_two_pillar_sequence_counts_two_clears(self):
        state = SlalomState()
        for _ in range(3):
            state.update("curve_around_obstacle", 0.5)
        for _ in range(RELEASE_FRAMES):
            state.update("advance", 0.0)
        for _ in range(3 + SIDE_COOLDOWN_FRAMES):
            state.update("curve_around_obstacle", -0.5)
        for _ in range(RELEASE_FRAMES):
            state.update("advance", 0.0)
        self.assertEqual(state.pillars_cleared, 2)
        self.assertEqual(state.side, 0)


class WatchdogTests(unittest.TestCase):
    def test_continuous_scanning_reaches_30_seconds(self):
        watchdog = ProgressWatchdog((0, 0), 0)
        for now in (0, 20, 29.9):
            watchdog.update(now, (0, 0), True)
        with self.assertRaisesRegex(TimeoutError, "30 seconds"):
            watchdog.update(30, (0, 0), True)

    def test_clear_flight_stall_times_out(self):
        watchdog = ProgressWatchdog((0, 0), 100)
        watchdog.update(119.9, (0, 0), False)
        with self.assertRaisesRegex(TimeoutError, "20 seconds"):
            watchdog.update(120, (0, 0), False)


class EventLogTests(unittest.TestCase):
    def test_repeated_runs_preserve_previous_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = SimpleNamespace(output=root, speed=1.2)
            first = create_run_directory(root, args)
            (first / "flight.csv").write_text("original evidence", encoding="utf-8")
            second = create_run_directory(root, args)
            self.assertNotEqual(first, second)
            self.assertEqual((first / "flight.csv").read_text(), "original evidence")

    def test_failure_and_cleanup_remain_separate_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            record_event(path, "timeout", "goal not reached")
            record_event(path, "landing_unconfirmed")
            events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([event["state"] for event in events],
                             ["timeout", "landing_unconfirmed"])


class ArgTests(unittest.TestCase):
    def test_default_goal_lies_beyond_second_pillar(self):
        # pillar_2 sits 32 m north of the spawn point in the documented
        # layout, so the default goal must be further than that or the
        # second pillar never enters view and the S path cannot be checked.
        args = parse_args([])
        self.assertGreater(args.goal_x, 32.0)
        self.assertEqual(args.output, Path("artifacts/double_pillar_avoidance"))
        self.assertAlmostEqual(args.speed, 1.2)

    def test_two_pillar_command_line_is_accepted(self):
        args = parse_args(["--route", "goal", "--goal-x", "38", "--goal-y", "0"])
        self.assertAlmostEqual(args.goal_x, 38.0)

    def test_out_of_range_speed_is_rejected(self):
        with self.assertRaises(SystemExit):
            parse_args(["--speed", "9"])


if __name__ == "__main__":
    unittest.main()
