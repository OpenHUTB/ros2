from concurrent.futures import Future
import math
import unittest

from uav_mission_safety_manager.controller import FlightController
from uav_mission_safety_manager.state_machine import State


class FakeAdapter:
    def __init__(self):
        self.calls = []
        self.reads = []

    def submit(self, command, **data):
        future = Future()
        self.calls.append((command, data, future))
        return future

    def sample(self):
        future = Future()
        self.reads.append(future)
        return future


class TestController(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.adapter = FakeAdapter()
        self.c = FlightController(self.adapter, clock=lambda: self.now)
        self.c.connected = True
        self.c.home = (20., 30., 2.)
        self.c.position = (40., 30., -8.)
        self.c.machine.state = State.EXECUTING

    def finish_command(self, result=None):
        self.adapter.calls[-1][2].set_result(result)
        self.c.tick()

    def observe(self, position, landed=False, velocity=None):
        self.c.tick()
        data = {'position': position, 'landed': landed}
        if velocity is not None:
            data['velocity'] = velocity
        self.adapter.reads[-1].set_result(data)
        self.c.tick()

    def test_return_uses_captured_home_and_lands_automatically(self):
        self.c.request('return')
        target = self.adapter.calls[-1][1]['target']
        self.assertEqual(target, (20., 30., -8.))
        self.finish_command()
        # Approach alone cannot trigger landing; require a stable stop.
        self.observe(target, velocity=(0., 0., 0.))
        self.finish_command()
        self.observe(target, velocity=(0., 0., 0.))
        self.now += 1.1
        self.finish_command()
        self.observe(target, velocity=(0., 0., 0.))
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.assertEqual(self.adapter.calls[-1][0], 'land')
        self.finish_command()
        # Ignore an in-flight sample taken before land dispatch.
        self.observe(target)
        self.observe(self.c.home, landed=True)
        self.assertEqual(self.adapter.calls[-1][0], 'disarm')
        self.finish_command()
        self.observe(self.c.home, landed=True)
        self.assertEqual(self.c.machine.state, State.LANDED)

    def test_invalid_land_has_no_external_effect(self):
        self.c.machine.state = State.IDLE
        with self.assertRaises(ValueError):
            self.c.request('land')
        self.assertEqual(self.adapter.calls, [])

    def test_connect_rejects_moving_vehicle_even_if_landed_flag_is_stale(self):
        self.c.machine.state = State.IDLE
        self.c.connected = False
        self.c.home = None
        self.c.request('connect')
        self.finish_command({'landed': True, 'position': (0., 0., 3.),
                             'velocity': (0., 0., 7.)})
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertIsNone(self.c.home)
        self.assertFalse(self.c.connected)

    def test_land_can_interrupt_return(self):
        self.c.request('return')
        self.finish_command()
        self.c.request('land')
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.assertEqual(self.adapter.calls[-1][0], 'land')

    def test_land_consumes_completed_precision_dispatch_before_next_timer(self):
        self.c.request('return')
        self.finish_command()
        self.observe((20.06, 30., -8.), velocity=(0., 0., 0.))
        self.assertEqual(self.adapter.calls[-1][0], 'velocity')
        self.c.tick()
        # The last status can still report pending even though its Future
        # completed. A service must consume it without waiting for the timer.
        self.adapter.calls[-1][2].set_result(None)
        self.adapter.reads[-1].set_result({
            'position': (20.04, 30., -8.), 'velocity': (0., 0., 0.), 'landed': False})
        self.assertEqual(self.c.machine.state, State.RETURNING)
        self.c.request('land')
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.assertEqual(self.adapter.calls[-1][0], 'land')

    def test_pending_rpc_does_not_block_tick_or_timeout(self):
        self.c.request('goto', x=30, y=30, z=-8)
        self.now = 3
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertFalse(self.adapter.calls[-1][2].done())

    def test_resume_reissues_target(self):
        self.c.request('goto', x=30, y=30, z=-8, speed=0.5)
        self.finish_command()
        self.c.request('pause')
        self.finish_command()
        self.c.request('resume')
        self.assertEqual(self.adapter.calls[-1][1]['target'], (30., 30., -8.))
        self.assertEqual(self.adapter.calls[-1][1]['speed'], 0.5)

    def test_return_corrects_offset_instead_of_landing_one_metre_away(self):
        self.c.request('return')
        self.finish_command()
        self.observe((20.47, 30., -8.), velocity=(0., 0., 0.))
        self.assertEqual(self.c.machine.state, State.RETURNING)
        command, data, _ = self.adapter.calls[-1]
        self.assertEqual(command, 'velocity')
        self.assertLess(data['velocity'][0], 0)
        self.assertLessEqual(math.sqrt(sum(v*v for v in data['velocity'])), 0.3)
        self.finish_command()
        self.observe((20., 30., -8.), velocity=(0.2, 0., 0.))
        self.assertEqual(self.c.machine.state, State.RETURNING)
        self.assertIsNone(self.c.return_settled_since)

    def test_return_stability_restarts_after_drift(self):
        self.c.request('return')
        self.finish_command()
        self.observe((20., 30., -8.), velocity=(0., 0., 0.))
        self.finish_command()
        self.observe((20., 30., -8.), velocity=(0., 0., 0.))
        self.now += 0.5
        self.finish_command()
        self.observe((20.2, 30., -8.), velocity=(0., 0., 0.))
        self.assertIsNone(self.c.return_settled_since)
        self.assertEqual(self.c.machine.state, State.RETURNING)

    def test_takeoff_false_is_recorded_without_inventing_ground_state(self):
        self.c.machine.state = State.TAKEOFF
        self.c.request('airborne')
        result = Future()
        self.finish_command(result)
        result.set_result(False)
        self.c.tick()
        self.assertIs(self.c.takeoff_rpc_result, False)
        self.assertIn('returned False', self.c.last_error)
        self.observe((20., 30., 0.), landed=False)
        self.assertEqual(self.c.machine.state, State.EXECUTING)

    def test_landing_rpc_error_does_not_claim_landed(self):
        self.c.request('land')
        self.adapter.calls[-1][2].set_exception(RuntimeError('disconnected'))
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertIn('disconnected', self.c.machine.reason)

    def test_takeoff_waits_for_observed_altitude(self):
        self.c.machine.state = State.TAKEOFF
        self.c.request('airborne')
        self.finish_command()
        self.observe(self.c.home, landed=True)
        self.assertEqual(self.c.machine.state, State.TAKEOFF)
        self.observe((20., 30., 0.), landed=False)
        self.assertEqual(self.c.machine.state, State.EXECUTING)

    def test_nonfinite_waypoint_rejected_before_dispatch(self):
        with self.assertRaises(ValueError):
            self.c.request('goto', x=float('nan'), y=0, z=-5)
        self.assertEqual(self.adapter.calls, [])

    def test_manual_completion_events_rejected_in_airsim_mode(self):
        for event in ('connected', 'at_home', 'landed'):
            with self.assertRaises(ValueError):
                self.c.request(event)

    def test_sample_before_land_request_cannot_complete_landing(self):
        self.c.tick()  # This read was started before the land request.
        old_read = self.adapter.reads[-1]
        self.c.request('land')
        self.finish_command()
        old_read.set_result({'position': self.c.home, 'landed': True})
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.observe(self.c.home, landed=True)
        self.finish_command()
        self.observe(self.c.home, landed=True)
        self.assertEqual(self.c.machine.state, State.LANDED)

    def start_landing_with_result(self, success=True):
        self.c.request('land')
        result = Future()
        self.finish_command(result)
        result.set_result(success)
        self.c.tick()
        # Drain the pre-land telemetry sample, which must be ignored.
        self.observe(self.c.home, velocity=(0., 0., 0.))

    def test_simpleflight_disarm_then_wait_for_fresh_landed_confirmation(self):
        self.start_landing_with_result()
        for i in range(5):
            self.now += 1
            self.observe(self.c.home, velocity=(0., 0., 0.))
            if self.c.pending:
                break
        self.assertEqual(self.adapter.calls[-1][0], 'disarm')
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.finish_command()
        self.observe(self.c.home, landed=False, velocity=(0., 0., 0.))
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.observe(self.c.home, landed=True, velocity=(0., 0., 0.))
        self.assertEqual(self.c.machine.state, State.LANDED)

    def test_landing_success_alone_never_disarms_in_air_or_away_from_home(self):
        self.start_landing_with_result()
        for position, velocity in [((20., 30., -3.), (0., 0., 0.)),
                                   ((25., 30., 2.), (0., 0., 0.)),
                                   (self.c.home, (0., 0., 0.2)),
                                   (self.c.home, None),
                                   (self.c.home, (float('nan'), 0., 0.))]:
            for i in range(5):
                self.now += 1
                self.observe(position, velocity=velocity)
        self.assertEqual([call[0] for call in self.adapter.calls], ['land'])

    def test_stationary_home_without_landing_success_never_disarms(self):
        self.c.request('land')
        self.finish_command(Future())
        for i in range(6):
            self.now += 1
            self.observe(self.c.home, velocity=(0., 0., 0.))
        self.assertEqual([call[0] for call in self.adapter.calls], ['land'])

    def test_failed_landing_result_enters_failsafe(self):
        self.start_landing_with_result(success=False)
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertEqual([call[0] for call in self.adapter.calls], ['land'])

    def test_disarm_failure_does_not_claim_landed(self):
        self.c.request('land')
        self.finish_command()
        self.observe(self.c.home, landed=True)
        self.assertEqual(self.adapter.calls[-1][0], 'disarm')
        self.adapter.calls[-1][2].set_exception(RuntimeError('disarm rejected'))
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.FAILSAFE)

    def test_telemetry_gap_restarts_stationary_confirmation(self):
        self.start_landing_with_result()
        self.now = 1
        self.observe(self.c.home, velocity=(0., 0., 0.))
        self.now = 5
        self.adapter.reads[-1].set_result({
            'position': self.c.home, 'landed': False, 'velocity': (0., 0., 0.)})
        self.c.tick()
        self.assertEqual(self.c.settled_since, 5)
        self.assertEqual(self.c.machine.state, State.LANDING)
        self.assertEqual([call[0] for call in self.adapter.calls], ['land'])

    def test_landing_requires_fresh_telemetry(self):
        self.c.request('land')
        self.finish_command()
        self.now = 3
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.FAILSAFE)

    def test_flight_operation_deadline_with_healthy_telemetry(self):
        self.c.request('return')
        self.finish_command()
        self.now = 121
        self.adapter.reads[-1].set_result({'position': (40., 30., -8.), 'landed': False})
        self.c.tick()
        self.assertEqual(self.c.machine.state, State.FAILSAFE)
        self.assertIn('Flight operation', self.c.machine.reason)


if __name__ == '__main__':
    unittest.main()
