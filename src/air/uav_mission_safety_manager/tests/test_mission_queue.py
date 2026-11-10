"""Queue acceptance, interruption and stale-telemetry regression tests."""
import unittest

from test_controller import FakeAdapter
from uav_mission_safety_manager.controller import FlightController
from uav_mission_safety_manager.state_machine import State


class TestMissionQueue(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.adapter = FakeAdapter()
        self.c = FlightController(self.adapter, clock=lambda: self.now)
        self.c.connected = True
        self.c.home = (0., 0., 0.)
        self.c.position = (0., 0., -5.)
        self.c.machine.state = State.EXECUTING
        self.points = [{'x': 10, 'y': 0, 'z': -5, 'speed': 0.5},
                       {'x': 20, 'y': 0, 'z': -5, 'speed': 1.5}]

    def start(self):
        self.c.request('mission', waypoints=self.points)
        self.finish()

    def finish(self):
        self.adapter.calls[-1][2].set_result(None)
        self.c.tick()

    def observe(self, position):
        # One read can predate the command revision; never reuse it to advance.
        for _ in range(2):
            self.c.tick()
            self.adapter.reads[-1].set_result({'position': position, 'landed': False})
            self.c.tick()

    def test_sequence_advances_with_per_waypoint_speed_and_finishes_hovering(self):
        self.start()
        self.assertEqual(self.adapter.calls[-1][1]['speed'], 0.5)
        self.observe((10., 0., -5.))
        self.assertEqual(self.adapter.calls[-1][1], {'target': (20., 0., -5.), 'speed': 1.5})
        self.assertEqual(self.c.mission_queue.completed, 1)
        self.finish()
        self.observe((20., 0., -5.))
        self.assertEqual(self.adapter.calls[-1][0], 'hover')
        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'COMPLETED', 'total': 2, 'completed': 2,
            'current_index': None, 'remaining': 0})
        self.assertIsNone(self.c.target)

    def test_invalid_tail_has_no_partial_dispatch_or_queue_change(self):
        invalids = [[], [None], [{}], self.points + [{'x': 0, 'y': 0, 'z': 1}],
                    [{'x': True, 'y': 0, 'z': -5}],
                    [{'x': float('nan'), 'y': 0, 'z': -5}],
                    [{'x': 10 ** 400, 'y': 0, 'z': -5}],
                    [{'x': 0, 'y': 0, 'z': -5, 'speed': 0}],
                    [{'x': 0, 'y': 0, 'z': -5, 'speed': float('inf')}],
                    [{'x': 0, 'y': 0, 'z': -51}],
                    [self.points[0]] * 101, 'not a list',
                    [dict(self.points[0], unknown=1)]]
        for points in invalids:
            with self.subTest(points=repr(points)[:80]):
                with self.assertRaises(ValueError):
                    self.c.request('mission', waypoints=points)
                self.assertEqual(self.adapter.calls, [])
                self.assertEqual(self.c.mission_queue.state, 'IDLE')

    def test_invalid_altitude_limits_are_rejected_before_dispatch(self):
        invalid_limits = [
            (float('nan'), 50),
            (float('inf'), 50),
            (0, float('inf')),
            (0, 0),
            (0, -1),
            (True, 50),
            (0, False),
            ('0', 50),
        ]
        for home_z, max_altitude in invalid_limits:
            with self.subTest(home_z=home_z, max_altitude=max_altitude):
                with self.assertRaises(ValueError):
                    self.c.mission_queue.load(self.points, home_z, max_altitude)
                self.assertEqual(self.c.mission_queue.state, 'IDLE')
                self.assertEqual(self.c.mission_queue.waypoints, ())

    def test_accepts_altitude_boundaries_and_the_100_waypoint_limit(self):
        home_z = 11.1961030960083
        max_altitude = 3.0
        waypoints = [
            {'x': index * 0.1, 'y': 0.0,
             'z': home_z - max_altitude if index == 0 else home_z - 1e-9}
            for index in range(100)]

        self.c.mission_queue.load(waypoints, home_z, max_altitude)

        self.assertEqual(self.c.mission_queue.state, 'RUNNING')
        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'RUNNING', 'total': 100, 'completed': 0,
            'current_index': 0, 'remaining': 100})
        self.assertEqual(self.c.mission_queue.waypoints[0],
                         ((0.0, 0.0, home_z - max_altitude), 3.0))

    def test_invalid_load_does_not_modify_an_existing_queue(self):
        self.c.mission_queue.load(self.points, 0.0, 50.0)
        before = (self.c.mission_queue.waypoints,
                  self.c.mission_queue.completed, self.c.mission_queue.state)

        with self.assertRaises(ValueError):
            self.c.mission_queue.load(
                [dict(self.points[0], z=0.0)], 0.0, 50.0)

        self.assertEqual((self.c.mission_queue.waypoints,
                          self.c.mission_queue.completed,
                          self.c.mission_queue.state), before)

    def test_pause_retains_index_and_resume_reissues_same_target(self):
        self.start()
        self.c.request('pause')
        self.finish()
        self.observe((10., 0., -5.))
        self.assertEqual(self.c.mission_queue.state, 'PAUSED')
        self.assertEqual(self.c.mission_queue.completed, 0)
        self.c.request('resume')
        self.assertEqual(self.adapter.calls[-1][1], {'target': (10., 0., -5.), 'speed': 0.5})
        self.assertEqual(self.c.mission_queue.state, 'RUNNING')

    def test_pause_status_reports_current_index_and_remaining(self):
        self.start()
        self.c.request('pause')
        self.finish()

        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'PAUSED', 'total': 2, 'completed': 0,
            'current_index': 0, 'remaining': 2})

    def test_cancel_discards_remaining_targets_even_after_resume(self):
        self.start()
        self.c.request('pause')
        self.finish()
        self.c.request('cancel')
        self.finish()
        self.c.request('resume')
        self.finish()
        self.observe((10., 0., -5.))
        self.assertEqual(self.c.mission_queue.state, 'CANCELED')
        self.assertEqual([c[0] for c in self.adapter.calls], ['goto', 'hover', 'hover', 'hover'])

    def test_cancel_status_is_terminal_and_does_not_dispatch_again(self):
        self.start()
        self.c.request('cancel')
        self.finish()
        calls = len(self.adapter.calls)

        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'CANCELED', 'total': 2, 'completed': 0,
            'current_index': None, 'remaining': 0})
        self.observe((10., 0., -5.))
        self.assertEqual(len(self.adapter.calls), calls)

    def test_cancel_after_progress_preserves_completed_count(self):
        self.start()
        self.observe((10., 0., -5.))
        self.finish()
        self.c.request('cancel')
        self.finish()

        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'CANCELED', 'total': 2, 'completed': 1,
            'current_index': None, 'remaining': 0})

    def test_return_and_land_preempt_queue(self):
        for event, state in [('return', State.RETURNING), ('land', State.LANDING)]:
            with self.subTest(event=event):
                self.setUp()
                self.start()
                self.c.request(event)
                self.assertEqual(self.c.machine.state, state)
                self.assertEqual(self.c.mission_queue.state, 'CANCELED')

    def test_goto_and_new_mission_cannot_silently_replace_queue(self):
        self.start()
        for event, data in [('goto', self.points[0]), ('mission', {'waypoints': self.points})]:
            with self.assertRaises(ValueError):
                self.c.request(event, **data)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_single_goto_must_finish_before_mission(self):
        self.c.request('goto', **self.points[0])
        self.finish()
        with self.assertRaises(ValueError):
            self.c.request('mission', waypoints=self.points)

    def test_stale_sample_cannot_skip_first_waypoint(self):
        self.c.tick()
        stale = self.adapter.reads[-1]
        self.start()
        stale.set_result({'position': (10., 0., -5.), 'landed': False})
        self.c.tick()
        self.assertEqual(self.c.mission_queue.completed, 0)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_rpc_error_aborts_without_dispatching_next_waypoint(self):
        self.c.request('mission', waypoints=self.points)
        self.adapter.calls[-1][2].set_exception(RuntimeError('RPC lost'))
        self.c.tick()
        self.assertEqual(self.c.mission_queue.state, 'ABORTED')
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertEqual(len(self.adapter.calls), 1)

    def test_abort_status_is_terminal_and_does_not_dispatch_again(self):
        self.c.request('mission', waypoints=self.points)
        self.adapter.calls[-1][2].set_exception(RuntimeError('RPC lost'))
        self.c.tick()
        calls = len(self.adapter.calls)

        self.assertEqual(self.c.mission_queue.status(), {
            'state': 'ABORTED', 'total': 2, 'completed': 0,
            'current_index': None, 'remaining': 0})
        self.observe((10., 0., -5.))
        self.assertEqual(len(self.adapter.calls), calls)

    def test_telemetry_timeout_aborts_queue(self):
        self.start()
        self.now = 3
        self.c.tick()
        self.assertEqual(self.c.mission_queue.state, 'ABORTED')
        self.observe((10., 0., -5.))
        self.assertEqual(len(self.adapter.calls), 1)

    def test_altitude_violation_aborts_queue(self):
        self.start()
        self.observe((0., 0., -60.))
        self.assertEqual(self.c.mission_queue.state, 'ABORTED')
        self.assertEqual(len(self.adapter.calls), 1)

    def test_expired_arrival_cannot_reset_leg_timeout(self):
        self.start()
        self.now = 121
        self.observe((10., 0., -5.))
        self.assertEqual(self.c.mission_queue.state, 'ABORTED')
        self.assertEqual(len(self.adapter.calls), 1)

    def test_cancel_without_mission_has_no_side_effects(self):
        with self.assertRaises(ValueError):
            self.c.request('cancel')
        self.assertEqual(self.adapter.calls, [])

    def test_new_mission_after_completion_restarts_progress(self):
        self.start()
        self.observe((10., 0., -5.))
        self.finish()
        self.observe((20., 0., -5.))
        self.finish()
        self.c.request('mission', waypoints=self.points[:1])
        self.assertEqual(self.c.mission_queue.completed, 0)
        self.assertEqual(len(self.c.mission_queue.waypoints), 1)


if __name__ == '__main__':
    unittest.main()
