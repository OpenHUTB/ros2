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
