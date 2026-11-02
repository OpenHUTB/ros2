"""Coordinate and stale-data checks independent of simulator availability."""

import importlib.util
import math
import json
import argparse
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'traffic_common', Path(__file__).parents[1] / 'tools' / 'traffic' / 'common.py')
common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(common)

MAIN_SPEC = importlib.util.spec_from_file_location(
    'traffic_main', Path(__file__).parents[1] / 'tools' / 'traffic' / 'main.py')
traffic_main = importlib.util.module_from_spec(MAIN_SPEC)
with patch.object(sys, 'path', [str(Path(__file__).parents[1] / 'tools' / 'traffic')] + sys.path):
    MAIN_SPEC.loader.exec_module(traffic_main)


class TrafficTrackingTests(unittest.TestCase):
    def test_follow_preserves_orientation_and_releases_without_hover(self):
        self.check_follow_control_state(False)

    def test_velocity_follow_uses_velocity_api_and_stops(self):
        self.check_follow_control_state(True, control_mode='velocity')

    def test_auto_mode_falls_back_to_pose_when_flight_controller_is_not_ready(self):
        self.check_follow_control_state(True, control_mode='auto', ready=False)

    def test_velocity_follow_stops_when_obstacle_route_is_blocked(self):
        self.check_follow_control_state(True, obstacle_mode='blocked',
                                        control_mode='velocity')

    def test_velocity_follow_refuses_to_take_off_when_landed(self):
        self.check_follow_control_state(False, control_mode='velocity', landed_state=0)

    def test_pose_follow_ignores_stale_landed_state_at_safe_height(self):
        self.check_follow_control_state(False, control_mode='pose', landed_state=0)

    def test_auto_pose_handles_stale_landed_state(self):
        self.check_follow_control_state(False, control_mode='auto', ready=False, landed_state=0)

    def test_pose_collision_override_does_not_exit_on_report(self):
        self.check_follow_control_state(False, collision_on_follow=True, ignore_collision=True)

    def test_pose_ground_guard_rejects_before_acquiring_control(self):
        self.check_follow_control_state(False, landed_state=0, pose_z=50)

    def test_pose_clearance_rejects_ground_and_nonfinite_coordinates(self):
        for z in (50, 49, math.nan):
            with self.assertRaisesRegex(RuntimeError, 'safe height'):
                common.pose_clearance([0, 0, z], {'z': 0}, [0, 0, 50])
        self.assertEqual(common.pose_clearance([0, 0, 10], {'z': 0}, [0, 0, 50]), 40)

    def test_live_read_retries_sharing_violation(self):
        snapshot = json.dumps({'time': time.time(), 'vehicles': []})
        with patch.object(Path, 'read_text', side_effect=[PermissionError(), snapshot]) as read:
            with patch.object(common.time, 'sleep'):
                self.assertEqual(common.read_live('.')['vehicles'], [])
            self.assertEqual(read.call_count, 2)

    def test_live_read_exhaustion_and_staleness_are_not_hidden(self):
        with patch.object(Path, 'read_text', side_effect=PermissionError()) as read:
            with patch.object(common.time, 'sleep'):
                with self.assertRaises(PermissionError):
                    common.read_live('.')
            self.assertEqual(read.call_count, 20)
        with patch.object(Path, 'read_text', side_effect=[PermissionError(),
                          json.dumps({'time': time.time() - 10})]):
            with patch.object(common.time, 'sleep'):
                with self.assertRaisesRegex(RuntimeError, 'stale'):
                    common.read_live('.')

    def test_velocity_follow_stops_on_collision(self):
        self.check_follow_control_state(True, collision_on_follow=True,
                                        control_mode='velocity')

    def test_follow_restores_native_keyboard_api_control(self):
        self.check_follow_control_state(True)

    def test_new_collision_stops_follow_and_restores_control(self):
        self.check_follow_control_state(True, collision_on_follow=True)

    def test_blocked_route_holds_position_and_restores_control(self):
        self.check_follow_control_state(True, obstacle_mode='blocked')

    def test_obstacle_query_failure_restores_control_without_pose_update(self):
        self.check_follow_control_state(True, obstacle_mode='failed')

    def check_follow_control_state(self, initial_api_control, collision_on_follow=False,
                                   obstacle_mode=None, control_mode='pose', landed_state=1,
                                   ready=None, ignore_collision=False, pose_z=25):
        if ready is None:
            ready = control_mode == 'velocity'
        effective_mode = ('velocity' if ready else 'pose') if control_mode == 'auto' else control_mode
        tools = str(Path(__file__).parents[1] / 'tools' / 'traffic')
        with patch.object(sys, 'path', [tools] + sys.path):
            import follow_vehicle
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            orientation = object()
            pose = SimpleNamespace(position=SimpleNamespace(x_val=0, y_val=0, z_val=pose_z),
                                   orientation=orientation)
            calls = []
            collision_reads = []

            class Client:
                def __init__(self, **kwargs):
                    self.api_enabled = initial_api_control

                def ping(self):
                    return True

                def simListSceneObjects(self):
                    return [str(i) for i in range(5)]

                def simGetObjectPose(self, name):
                    return SimpleNamespace(position=SimpleNamespace(x_val=0, y_val=0, z_val=50))

                def simGetVehiclePose(self):
                    return pose

                def getMultirotorState(self):
                    return SimpleNamespace(collision=SimpleNamespace(has_collided=False),
                                           landed_state=landed_state, ready=ready)

                def simGetCollisionInfo(self):
                    collision_reads.append(True)
                    return SimpleNamespace(has_collided=True, object_name='test_wall',
                                           time_stamp=101 if collision_on_follow
                                           and len(collision_reads) > 1 else 100)

                def cancelLastTask(self):
                    calls.append('cancel')

                def enableApiControl(self, enabled):
                    calls.append(('api', enabled))
                    self.api_enabled = enabled

                def isApiControlEnabled(self):
                    return self.api_enabled

                def simSetVehiclePose(self, updated, ignore_collision):
                    self_test.assertEqual(effective_mode, 'pose')
                    self_test.assertIs(updated.orientation, orientation)
                    self_test.assertEqual(ignore_collision, args.ignore_collision)
                    if obstacle_mode == 'blocked':
                        self_test.assertEqual(updated.position, (0, 0, 25))
                    calls.append('pose')
                    (root / 'follow_exit').touch()

                def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain, yaw_mode):
                    self_test.assertEqual(effective_mode, 'velocity')
                    calls.append(('velocity', vx, vy, vz, duration, drivetrain, yaw_mode))
                    (root / 'follow_exit').touch()
                    return SimpleNamespace(join=lambda: calls.append('velocity_join'))

            rejected_landed = landed_state == 0 and effective_mode == 'velocity'
            rejected_height = effective_mode == 'pose' and 50 - pose_z < 2
            rejected_collision = collision_on_follow and not ignore_collision
            self_test = self
            vendor = ModuleType('openhutb_yolo_mlp_control.vendor')
            vendor.airsim = SimpleNamespace(
                MultirotorClient=Client, Vector3r=lambda *v: v,
                Pose=lambda position, orientation: SimpleNamespace(position=position, orientation=orientation),
                DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom='max'),
                YawMode=lambda is_rate, angle: (is_rate, angle),
                LandedState=SimpleNamespace(Flying=1, Landed=0))
            common.write_json(root / 'references.json', {'map': 'test', 'references':
                              [{'name': str(i), 'carla': [0, 0, 0]} for i in range(5)]})
            common.write_json(root / 'live.json', {'time': time.time(), 'map': 'test', 'vehicles':
                              [dict(actor_id=1, x=0, y=0, z=0, yaw_deg=0,
                                    vx=2, vy=0, vz=0)]})
            args = SimpleNamespace(data=root, host='fake', port=41451, target_id=None,
                                   distance=12, height=25, speed=12, duration=90,
                                   ignore_collision=ignore_collision,
                                   avoid_obstacles=obstacle_mode is not None,
                                   obstacle_host=None, obstacle_port=41452, obstacle_margin=1.5,
                                   control_mode=control_mode)
            def query(rays):
                if obstacle_mode == 'failed':
                    raise RuntimeError('Scene query failed')
                return False
            queries = SimpleNamespace(clear=query, close=lambda: None)
            with patch.dict(sys.modules, {'openhutb_yolo_mlp_control.vendor': vendor}):
                with patch.object(follow_vehicle.time, 'sleep'), patch.object(
                        follow_vehicle, 'SceneQueries', return_value=queries):
                    if rejected_landed:
                        with self.assertRaisesRegex(RuntimeError, 'will not take off'):
                            follow_vehicle.follow(args)
                    elif rejected_height:
                        with self.assertRaisesRegex(RuntimeError, 'safe height'):
                            follow_vehicle.follow(args)
                    elif rejected_collision:
                        with self.assertRaisesRegex(RuntimeError, 'Collision with test_wall'):
                            follow_vehicle.follow(args)
                    elif obstacle_mode == 'failed':
                        with self.assertRaisesRegex(RuntimeError, 'Scene query failed'):
                            follow_vehicle.follow(args)
                    else:
                        follow_vehicle.follow(args)
            if rejected_landed or rejected_height:
                self.assertEqual(calls, [])
            elif effective_mode == 'velocity':
                self.assertEqual(calls[:2], [('api', True), 'cancel'])
                velocity_calls = [call for call in calls if isinstance(call, tuple)
                                  and call[0] == 'velocity']
                successful_step = not rejected_collision and obstacle_mode != 'failed'
                self.assertEqual(len(velocity_calls), 2 if successful_step else 1)
                if successful_step:
                    self.assertLessEqual(math.sqrt(sum(value * value
                                                       for value in velocity_calls[0][1:4])),
                                          args.speed)
                    if obstacle_mode == 'blocked':
                        self.assertEqual(velocity_calls[0][1:4], (0.0, 0.0, 0.0))
                self.assertEqual(velocity_calls[-1][1:4], (0.0, 0.0, 0.0))
                self.assertEqual(calls[-3:], ['velocity_join', 'cancel',
                                              ('api', initial_api_control)])
            else:
                self.assertEqual(calls, [('api', True), 'cancel']
                                 + ([] if rejected_collision or obstacle_mode == 'failed' else ['pose'])
                                 + ['cancel', ('api', initial_api_control)])
            status = json.loads((root / 'follow_status.json').read_text())
            self.assertTrue(status['control_released'])
            self.assertTrue(status['api_control_restored'])
            self.assertEqual(status['final_api_control'], initial_api_control)
            self.assertEqual(status['reason'], 'collision' if rejected_collision else
                             'error' if obstacle_mode == 'failed' or rejected_landed or rejected_height
                             else 'manual_exit')
            if obstacle_mode == 'blocked':
                import csv
                with (root / 'drone_follow.csv').open() as stream:
                    self.assertEqual(next(csv.DictReader(stream))['phase'], 'blocked')

    def test_calibration_flips_z_and_fits_translation(self):
        pairs = [([i, i * 2, i * 3], [i - 36, i * 2 + 7, 50 - i * 3]) for i in range(6)]
        offset, residual = common.calibration_offset(pairs)
        self.assertEqual(offset, [-36, 7, 50])
        self.assertEqual(residual, 0)

    def test_wrong_reference_and_nonfinite_are_rejected(self):
        pairs = [([0, 0, 0], [1, 2, 3]) for _ in range(5)]
        pairs[-1] = ([0, 0, 0], [9, 2, 3])
        with self.assertRaises(ValueError):
            common.calibration_offset(pairs)
        pairs[-1] = ([0, 0, 0], [math.nan, 2, 3])
        with self.assertRaises(ValueError):
            common.calibration_offset(pairs)

    def test_following_height_and_trailing_distance(self):
        point = common.following_point(dict(x=10, y=20, z=1, yaw_deg=90), [-36, 7, 50], 12, 25)
        for actual, expected in zip(point, [-26, 15, 24]):
            self.assertAlmostEqual(actual, expected)

    def test_step_is_limited_and_does_not_overshoot(self):
        result = common.step_towards([0, 0, 0], [3, 4, 0], 2)
        for actual, expected in zip(result, [1.2, 1.6, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(common.step_towards([0, 0, 0], [3, 4, 0], 20), [3, 4, 0])

    def test_velocity_command_is_speed_and_acceleration_limited(self):
        command = common.velocity_towards([0, 0, 0], [100, 0, 0], 12,
                                          previous=[0, 0, 0], dt=0.2)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in command)), 0.6)
        command = common.velocity_towards([0, 0, 0], [100, 0, 0], 12,
                                          previous=[11, 0, 0], dt=0.2)
        self.assertLessEqual(math.sqrt(sum(v * v for v in command)), 12)
        self.assertAlmostEqual(command[0], 11.6)

    def test_velocity_command_brakes_near_waypoint(self):
        command = common.velocity_towards([0, 0, 0], [1, 0, 0], 12,
                                          previous=[0, 0, 0], dt=0.2)
        self.assertLess(command[0], 2)

    def test_velocity_command_tracks_target_motion_with_feedforward(self):
        command = common.velocity_towards([0, 0, 0], [0, 0, 0], 12,
                                          previous=[0, 0, 0], dt=0.2,
                                          target_velocity=[8, 0, 0])
        self.assertAlmostEqual(command[0], 0.6)
        self.assertEqual(command[1:], [0.0, 0.0])

    def test_positive_rejects_nonfinite_and_nonpositive_values(self):
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(argparse.ArgumentTypeError):
                common.positive(value)

    def test_follow_phase_keeps_horizontal_pursuit_during_climb(self):
        current = [0, 0, 0]
        target = [15, -8, -25]
        self.assertEqual(common.follow_phase(current, target), 'climb')
        self.assertEqual(target[:2], [15, -8])
        self.assertEqual(common.follow_phase([15, -8, -24], target), 'follow')

    def test_climb_allocates_vertical_progress_with_distant_target(self):
        current = [0, 0, 0]
        target = [1000, 0, -25]
        result = common.follow_step(current, target, 2)
        self.assertGreater(result[0], 0)
        self.assertLess(result[2], -1.4)
        self.assertLessEqual(math.dist(current, result), 2.00000001)
        self.assertEqual(common.follow_step([0, 0, -24], [0, 0, -25], 2), [0, 0, -25])

    def test_actual_sampling_rate_uses_elapsed_sample_interval(self):
        self.assertAlmostEqual(common.actual_sampling_rate(2200, 30.05), 2199 / 30.05)
        self.assertEqual(common.actual_sampling_rate(1, 1.0), 0.0)

    def test_calibration_reference_export_is_bounded(self):
        references = [{'name': str(index)} for index in range(2005)]
        selected = common.limited_references(references)
        self.assertEqual(len(selected), 20)
        self.assertEqual(selected[0]['name'], '0')
        self.assertEqual(selected[-1]['name'], '19')

    def test_clean_removes_only_tool_managed_vehicles(self):
        class Actor:
            def __init__(self, role):
                self.attributes = {'role_name': role}
                self.destroyed = False

            def destroy(self):
                self.destroyed = True
                return True

        class Actors(list):
            def filter(self, pattern):
                self_test.assertEqual(pattern, 'vehicle.*')
                return self

        managed = Actor(traffic_main.ROLE)
        unrelated = Actor('other')
        self_test = self
        world = SimpleNamespace(get_actors=lambda: Actors([managed, unrelated]))
        with patch.object(traffic_main, 'connect', return_value=(None, world)):
            traffic_main.clean(SimpleNamespace(host='fake', port=2000))
        self.assertTrue(managed.destroyed)
        self.assertFalse(unrelated.destroyed)

    def test_stale_and_future_telemetry_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'live.json'
            for age in (3, -3):
                common.write_json(path, {'time': time.time() - age})
                with self.assertRaises(RuntimeError):
                    common.read_live(directory)
            common.write_json(path, {'time': time.time(), 'vehicles': []})
            self.assertEqual(common.read_live(directory)['vehicles'], [])


if __name__ == '__main__':
    unittest.main()
