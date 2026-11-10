"""Synthetic depth and state-machine tests; no AirSim server required."""
import json
import math
import tempfile
import unittest
from contextlib import redirect_stderr
from io import StringIO
from pathlib import Path
from types import SimpleNamespace

from avoidance import (GROUND_ROWS_DEFAULT, MAX_YAW_RATE, OVERFLY_DONE_FRAMES,
                       OVERFLY_HOLD_FRAMES, OVERFLY_MAX_FAILS, OverflyState,
                       ProgressWatchdog, analyze_columns, camera_geometry,
                       check_new_collision, committed_climb, create_run_directory,
                       decode_unity_depth, flip_rows, parse_args, plan_height_aware,
                       record_event, route_from_start, smooth_yaw_rate)

WIDTH, HEIGHT, FOV = 160, 90, 90.0
FREE = 50.0


def blank_frame():
    return [FREE] * (WIDTH * HEIGHT)


def paint(image, distance, top_height, col_from=0, col_to=WIDTH - 1):
    """Fill a vertical surface from its top edge down to the ground band."""
    focal, centre, row_top, row_bottom = camera_geometry(
        WIDTH, HEIGHT, FOV, GROUND_ROWS_DEFAULT)
    top_row = max(row_top, int(round(centre - top_height * focal / distance)))
    for row in range(top_row, row_bottom + 1):
        for col in range(col_from, col_to + 1):
            image[row * WIDTH + col] = distance
    return image


def analyze(image):
    return analyze_columns(image, WIDTH, HEIGHT, FOV)


def command(profiles, goal=0.0, max_climb=3.5, max_altitude=7.0,
            cruise=3.0, armed=True, stop=3.0, offset=0.0):
    return plan_height_aware(profiles, WIDTH, HEIGHT, FOV, goal, 1.2, stop, 1.5, 0.6,
                             0.0, 0, 2.0, max_climb, max_altitude, cruise, armed,
                             camera_offset=offset)


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


class DepthDecodeTests(unittest.TestCase):
    def test_unity_depth_is_decoded_to_metres(self):
        encoded = 0.5 ** 0.25
        decoded = decode_unity_depth([encoded], near=0.3, far=100.0)[0]
        self.assertAlmostEqual(decoded, (0.5 / 1.003 + 0.003) * 100, places=2)

    def test_srgb_depth_round_trip(self):
        for distance in (0.5, 1, 2, 3, 10):
            shader = ((distance / 1000 - 0.3 / 1000) * 1.0003) ** 0.25
            captured = 1.055 * shader ** (1 / 2.4) - 0.055
            decoded = decode_unity_depth([captured], srgb=True)[0]
            self.assertAlmostEqual(decoded, distance, places=5)

    def test_invalid_pixels_stay_invalid(self):
        decoded = decode_unity_depth([float("nan"), 0.0, 1.5, -1.0])
        self.assertTrue(all(math.isnan(value) for value in decoded))

    def test_bad_calibration_is_rejected(self):
        with self.assertRaises(ValueError):
            decode_unity_depth([0.5], near=10.0, far=1.0)
        with self.assertRaises(ValueError):
            decode_unity_depth([0.5], compression=0.0)

    def test_bottom_up_unity_buffer_is_flipped_to_top_down(self):
        width, height = 3, 2
        bottom_up = [1, 2, 3, 4, 5, 6]  # row 0 is the bottom of the image
        flipped = flip_rows(bottom_up, width, height)
        self.assertEqual(flipped, [4, 5, 6, 1, 2, 3])
        with self.assertRaises(ValueError):
            flip_rows([1, 2, 3], width, height)


class CameraTests(unittest.TestCase):
    def test_geometry_matches_pinhole_focal_length(self):
        # AirSim Unity reports the vertical FOV, so the focal length comes from
        # the image height; the width would be the Unreal (horizontal) case.
        focal, centre, row_top, row_bottom = camera_geometry(WIDTH, HEIGHT, FOV, 12)
        self.assertAlmostEqual(focal, HEIGHT / (2 * math.tan(math.radians(FOV) / 2)))
        self.assertAlmostEqual(centre, (HEIGHT - 1) / 2)
        self.assertEqual(row_top, HEIGHT // 8)
        self.assertEqual(row_bottom, HEIGHT - 1 - 12)


class ColumnTests(unittest.TestCase):
    def test_wall_top_edge_is_reconstructed_above_camera(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        centre = profiles[WIDTH // 2]
        self.assertTrue(centre.valid)
        self.assertAlmostEqual(centre.distance, 6.0)
        self.assertAlmostEqual(centre.top_height, 1.0, delta=0.1)
        self.assertGreater(centre.overhead, 1.0)

    def test_ground_rows_are_excluded(self):
        image = blank_frame()
        for row in range(HEIGHT - 6, HEIGHT):
            for col in range(WIDTH):
                image[row * WIDTH + col] = 1.0
        profiles = analyze(image)
        self.assertEqual(command(profiles).mode, "advance")

    def test_image_length_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze_columns([1.0] * 10, WIDTH, HEIGHT, FOV)

    def test_all_invalid_pixels_stop_the_mission(self):
        with self.assertRaisesRegex(ValueError, "No valid depth pixels"):
            analyze([float("nan")] * (WIDTH * HEIGHT))

    def test_dead_pixel_reads_as_zero_range(self):
        image = blank_frame()
        image[40 * WIDTH + 80] = float("nan")
        profiles = analyze(image)
        self.assertEqual(profiles[80].distance, 0.0)
        self.assertEqual(command(profiles).mode, "blocked")


class PlannerTests(unittest.TestCase):
    def test_advance_when_corridor_is_clear(self):
        cmd = command(analyze(blank_frame()))
        self.assertEqual(cmd.mode, "advance")
        self.assertAlmostEqual(cmd.speed, 1.2)

    def test_low_wall_triggers_overfly_with_full_body_margin(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        cmd = command(profiles)
        self.assertEqual(cmd.reason, "overfly_obstacle")
        self.assertGreater(cmd.climb, 0.0)
        self.assertGreater(cmd.speed, 0.0)
        # Committed height is the top edge plus the inflated body radius.
        self.assertAlmostEqual(cmd.top_height + 2.1, 3.11, delta=0.15)

    def test_obstacle_below_camera_needs_no_climb(self):
        # A wall whose top is under the camera by more than the body radius is
        # already clear; the committed climb must clamp to zero, never negative.
        profiles = analyze(paint(blank_frame(), 6.0, -2.4))
        cmd = command(profiles)
        self.assertEqual(cmd.mode, "overfly")
        self.assertLess(cmd.top_height, 0.0)
        self.assertEqual(cmd.climb, 0.0)

    def test_tall_wall_bypasses_instead_of_climbing(self):
        profiles = analyze(paint(blank_frame(), 6.0, 2.0))
        cmd = command(profiles)
        self.assertEqual(cmd.reason, "curve_around_obstacle")
        self.assertEqual(cmd.climb, 0.0)

    def test_disarmed_overfly_degrades_to_lateral_bypass(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        self.assertEqual(command(profiles, armed=False).reason, "curve_around_obstacle")

    def test_climb_budget_bounds_the_overfly_decision(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        self.assertEqual(command(profiles, max_climb=3.0).reason, "curve_around_obstacle")
        self.assertEqual(command(profiles, max_climb=3.5).reason, "overfly_obstacle")

    def test_altitude_ceiling_bounds_the_overfly_decision(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        self.assertEqual(command(profiles, max_altitude=5.0).reason, "curve_around_obstacle")
        self.assertEqual(command(profiles, max_altitude=7.0).reason, "overfly_obstacle")

    def test_unverified_top_never_triggers_a_climb(self):
        # The top edge is above the analysed band, so overhead is zero even
        # though the altitude budget would otherwise allow the climb.
        profiles = analyze(paint(blank_frame(), 6.0, 5.0))
        self.assertEqual(command(profiles, max_climb=8.0).reason, "curve_around_obstacle")

    def test_side_obstacle_inside_body_radius_vetoes_overfly(self):
        # Low wall ahead plus a taller pillar 1.8 m to the side, 4.5 m away:
        # the pillar is outside the central slice but inside the inflated body.
        image = paint(blank_frame(), 6.0, 1.0)
        paint(image, 4.5, 1.5, col_from=95, col_to=101)
        self.assertEqual(command(analyze(image)).reason, "curve_around_obstacle")

    def test_obstacle_outside_body_radius_keeps_advance(self):
        # A wall only on the far left at 4 m is farther than the 2.1 m radius.
        image = paint(blank_frame(), 4.0, 1.0, col_from=0, col_to=20)
        self.assertEqual(command(analyze(image)).mode, "advance")

    def test_invalid_planner_settings_are_rejected(self):
        profiles = analyze(blank_frame())
        with self.assertRaises(ValueError):
            plan_height_aware(profiles, WIDTH, HEIGHT, 20.0, 0)
        with self.assertRaises(ValueError):
            plan_height_aware(profiles[:10], WIDTH, HEIGHT, FOV, 0)
        with self.assertRaises(ValueError):
            plan_height_aware(profiles, WIDTH, HEIGHT, FOV, 0, free_distance=2.0)
        with self.assertRaises(ValueError):
            plan_height_aware(profiles, WIDTH, HEIGHT, FOV, 0, camera_offset=-0.1)

    def test_camera_offset_counts_against_the_climb_budget(self):
        # Top 0.8 m above the camera: 2.9 m needed without the mount offset
        # (fits max-climb 3.0), 3.39 m with the measured 0.49 m offset (does not).
        profiles = analyze(paint(blank_frame(), 6.0, 0.8))
        self.assertEqual(command(profiles, max_climb=3.0).reason, "overfly_obstacle")
        self.assertEqual(command(profiles, max_climb=3.0, offset=0.49).reason,
                         "curve_around_obstacle")

    def test_camera_offset_increases_the_needed_climb(self):
        profiles = analyze(paint(blank_frame(), 6.0, 1.0))
        base = committed_climb(
            max(p.top_height for p in profiles if p.valid), 0.0, 2.1)
        shifted = committed_climb(
            max(p.top_height for p in profiles if p.valid), 0.49, 2.1)
        self.assertAlmostEqual(shifted - base, 0.49, places=6)


class CommittedClimbTests(unittest.TestCase):
    def test_offset_is_added_to_the_committed_climb(self):
        self.assertAlmostEqual(committed_climb(0.04, 0.49, 2.1), 2.63, places=6)

    def test_below_camera_clamps_to_zero(self):
        self.assertEqual(committed_climb(-3.0, 0.49, 2.1), 0.0)

    def test_zero_offset_matches_the_old_formula(self):
        self.assertAlmostEqual(committed_climb(1.0, 0.0, 2.1), 3.1, places=6)




class OverflyStateTests(unittest.TestCase):
    def test_commit_requires_stable_target_for_three_frames(self):
        state = OverflyState()
        for _ in range(OVERFLY_HOLD_FRAMES - 1):
            self.assertFalse(state.request(3.0))
        self.assertTrue(state.request(3.0))
        self.assertTrue(state.active)
        self.assertEqual(state.climb_target, 3.0)

    def test_disarmed_state_never_commits(self):
        state = OverflyState(armed=False)
        for _ in range(10):
            self.assertFalse(state.request(3.0))
        self.assertFalse(state.active)

    def test_committed_climb_is_never_lowered(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        state.request(1.0)
        self.assertEqual(state.climb_target, 3.0)

    def test_extension_restarts_the_progress_window(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        self.assertFalse(state.request(3.1))
        self.assertTrue(state.request(3.4))
        self.assertEqual(state.climb_target, 3.4)

    def test_six_clear_frames_complete_the_overfly(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        for _ in range(OVERFLY_DONE_FRAMES - 1):
            self.assertFalse(state.note_clear())
        self.assertTrue(state.note_clear())
        self.assertFalse(state.active)
        self.assertEqual(state.completed, 1)
        self.assertEqual(state.climb_target, 0.0)

    def test_clear_streak_does_not_survive_a_bypass(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        for _ in range(OVERFLY_DONE_FRAMES - 1):
            state.note_clear()
        state.suspend_clear()
        for _ in range(OVERFLY_DONE_FRAMES - 1):
            self.assertFalse(state.note_clear())
        self.assertTrue(state.active)
        self.assertEqual(state.completed, 0)

    def test_two_failed_climbs_disarm_overfly(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        for _ in range(OVERFLY_MAX_FAILS):
            state.fail()
        self.assertFalse(state.armed)
        self.assertFalse(state.active)
        self.assertEqual(state.climb_started, 0.0)

    def test_completion_starts_a_holdover(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=1.0)
        for _ in range(OVERFLY_DONE_FRAMES - 1):
            self.assertFalse(state.note_clear())
        held = state.climb_target
        self.assertTrue(state.note_clear())
        state.begin_hold(held, (20.0, -320.0))
        self.assertTrue(state.hold)
        self.assertEqual(state.hold_altitude, 3.0)
        self.assertEqual(state.hold_start, (20.0, -320.0))

    def test_hold_end_and_cancel_reset_the_state(self):
        state = OverflyState()
        state.begin_hold(3.0, (20.0, -320.0))
        state.end_hold()
        self.assertFalse(state.hold)
        self.assertEqual(state.hold_altitude, 0.0)

    def test_new_climb_cancels_an_active_holdover(self):
        state = OverflyState()
        state.begin_hold(3.0, (20.0, -320.0))
        for _ in range(OVERFLY_HOLD_FRAMES):
            state.request(2.0)
        self.assertTrue(state.active)
        self.assertFalse(state.hold)

    def test_failed_climb_cancels_an_active_holdover(self):
        state = OverflyState()
        state.begin_hold(3.0, (20.0, -320.0))
        state.fail()
        self.assertFalse(state.hold)

    def test_progress_is_only_judged_after_the_grace_period(self):
        state = OverflyState(active=True, climb_target=3.0, climb_started=100.0)
        self.assertFalse(state.check_progress(105.0, 0.0))
        self.assertTrue(state.check_progress(110.0, 1.0))
        self.assertFalse(state.check_progress(110.0, 2.1))

    def test_inactive_state_never_reports_failure(self):
        state = OverflyState(armed=True)
        self.assertFalse(state.check_progress(1000.0, 0.0))


class WatchdogTests(unittest.TestCase):
    def test_progress_uses_position_not_elapsed_time(self):
        watchdog = ProgressWatchdog((0.0, 0.0), 0.0)
        watchdog.update(5.0, (1.0, 0.0), False)
        watchdog.update(10.0, (1.0, 0.0), False)
        self.assertEqual(watchdog.progress_time, 5.0)

    def test_stalled_progress_raises(self):
        watchdog = ProgressWatchdog((0.0, 0.0), 0.0)
        with self.assertRaisesRegex(TimeoutError, "No horizontal progress"):
            watchdog.update(20.0, (0.3, 0.0), False)

    def test_blocked_scanning_has_its_own_timeout(self):
        watchdog = ProgressWatchdog((0.0, 0.0), 0.0)
        watchdog.update(0.0, (0.0, 0.0), True)
        with self.assertRaisesRegex(TimeoutError, "No clear corridor"):
            watchdog.update(30.0, (0.0, 0.0), True)

    def test_clear_frame_restarts_the_scan_clock(self):
        watchdog = ProgressWatchdog((0.0, 0.0), 0.0)
        watchdog.update(0.0, (0.0, 0.0), True)
        watchdog.update(10.0, (0.0, 0.0), False)
        self.assertIsNone(watchdog.blocked_since)


class RunDirectoryTests(unittest.TestCase):
    def test_each_run_gets_an_isolated_directory_and_settings(self):
        args = parse_args([])
        with tempfile.TemporaryDirectory() as root:
            first = create_run_directory(Path(root), args)
            second = create_run_directory(Path(root), args)
            self.assertNotEqual(first, second)
            settings = json.loads((first / "settings.json").read_text(encoding="utf-8"))
            self.assertEqual(settings["altitude"], args.altitude)
            self.assertEqual(settings["max_climb"], args.max_climb)

    def test_events_append_as_json_lines(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "events.jsonl"
            record_event(path, "mission_started")
            record_event(path, "goal_reached", "ok")
            lines = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual([json.loads(line)["state"] for line in lines],
                             ["mission_started", "goal_reached"])
            self.assertEqual(json.loads(lines[1])["reason"], "ok")


class CommandTests(unittest.TestCase):
    def test_rectangle_route_returns_four_corners(self):
        self.assertEqual(route_from_start(10.0, 20.0, 30.0, 40.0),
                         [(40.0, 20.0), (40.0, 60.0), (10.0, 60.0), (10.0, 20.0)])

    def test_yaw_rate_change_is_rate_limited(self):
        self.assertAlmostEqual(smooth_yaw_rate(0.0, 20.0, 0.1), 1.2)
        self.assertAlmostEqual(smooth_yaw_rate(0.0, -20.0, 0.5), -0.5 * MAX_YAW_RATE)


class ArgumentTests(unittest.TestCase):
    def reject(self, argv):
        with self.assertRaises(SystemExit), redirect_stderr(StringIO()):
            parse_args(argv)

    def test_defaults_are_self_consistent(self):
        args = parse_args([])
        self.assertGreater(args.max_altitude, args.altitude)
        self.assertGreaterEqual(args.max_climb, 2.1)
        self.assertGreater(args.max_altitude - args.altitude, args.max_climb - 0.5)
        self.assertEqual(args.overfly_hold_distance, 5.0)

    def test_negative_hold_distance_is_rejected(self):
        self.reject(["--overfly-hold-distance", "-1"])

    def test_zero_hold_distance_is_allowed(self):
        args = parse_args(["--overfly-hold-distance", "0"])
        self.assertEqual(args.overfly_hold_distance, 0.0)

    def test_invalid_speed_is_rejected(self):
        self.reject(["--speed", "2.0"])

    def test_ceiling_must_exceed_cruise_altitude(self):
        self.reject(["--max-altitude", "3.0"])

    def test_stop_distance_must_be_safe(self):
        self.reject(["--stop-distance", "1.0"])


if __name__ == "__main__":
    unittest.main()
